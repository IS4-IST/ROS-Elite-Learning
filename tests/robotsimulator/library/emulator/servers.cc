#include "emulator/servers.h"

#include <cstdio>
#include <istream>

namespace emu {

// --- 8055 -----------------------------------------------------------------------
struct RpcServer::Session : std::enable_shared_from_this<RpcServer::Session> {
    Session(asio::ip::tcp::socket s, Controller& c) : socket(std::move(s)), controller(c) {}

    void start() { read(); }

    void read() {
        auto self = shared_from_this();
        asio::async_read_until(socket, buffer, '\n', [this, self](std::error_code ec, std::size_t) {
            if (ec) return;   // client disconnected
            std::istream in(&buffer);
            std::string line;
            std::getline(in, line);
            if (!line.empty() && line.find_first_not_of(" \r\t") != std::string::npos) {
                auto request = nlohmann::json::parse(line, nullptr, /*allow_exceptions=*/false);
                std::optional<nlohmann::json> reply;
                if (request.is_discarded() || !request.is_object()) {
                    reply = nlohmann::json{{"jsonrpc", "2.0"}, {"id", nullptr},
                                           {"error", {{"code", -32700}, {"message", "Parse error"}}}};
                } else {
                    reply = controller.handle_request(request);
                }
                if (reply) write(reply->dump() + "\n");
            }
            read();
        });
    }

    void write(std::string text) {
        queue.push_back(std::move(text));
        if (queue.size() == 1) flush();
    }

    void flush() {
        auto self = shared_from_this();
        asio::async_write(socket, asio::buffer(queue.front()), [this, self](std::error_code ec, std::size_t) {
            if (ec) return;
            queue.pop_front();
            if (!queue.empty()) flush();
        });
    }

    asio::ip::tcp::socket socket;
    Controller& controller;
    asio::streambuf buffer;
    std::deque<std::string> queue;
};

RpcServer::RpcServer(asio::io_context& io, Controller& controller, const std::string& bind, unsigned short port)
    : io_(io), controller_(controller), acceptor_(io) {
    asio::ip::tcp::endpoint ep(asio::ip::make_address(bind), port);
    acceptor_.open(ep.protocol());
    acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true));
    acceptor_.bind(ep);
    acceptor_.listen();
    accept();
}

bool RpcServer::connected() const {
    auto current = active_.lock();
    return current && current->socket.is_open();
}

void RpcServer::accept() {
    acceptor_.async_accept([this](std::error_code ec, asio::ip::tcp::socket socket) {
        if (!ec) {
            if (auto current = active_.lock(); current && current->socket.is_open()) {
                // the real controller serves one client on 8055
                std::error_code ignored;
                socket.close(ignored);
                std::fprintf(stderr, "[8055] refused a second client\n");
            } else {
                socket.set_option(asio::ip::tcp::no_delay(true));
                auto session = std::make_shared<Session>(std::move(socket), controller_);
                active_ = session;
                session->start();
            }
        }
        accept();
    });
}

// --- 8056 -----------------------------------------------------------------------
MonitorServer::MonitorServer(asio::io_context& io, const std::string& bind, unsigned short port) : acceptor_(io) {
    asio::ip::tcp::endpoint ep(asio::ip::make_address(bind), port);
    acceptor_.open(ep.protocol());
    acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true));
    acceptor_.bind(ep);
    acceptor_.listen();
    accept();
}

void MonitorServer::accept() {
    acceptor_.async_accept([this](std::error_code ec, asio::ip::tcp::socket socket) {
        if (!ec) {
            socket.set_option(asio::ip::tcp::no_delay(true));
            clients_.push_back(std::make_shared<Client>(std::move(socket)));
        }
        accept();
    });
}

void MonitorServer::broadcast(std::vector<std::uint8_t> packet) {
    auto data = std::make_shared<std::vector<std::uint8_t>>(std::move(packet));
    for (auto it = clients_.begin(); it != clients_.end();) {
        auto client = *it;
        if (!client->socket.is_open()) {
            it = clients_.erase(it);
            continue;
        }
        if (!client->writing) {     // a slow client skips packets instead of piling them up
            client->writing = true;
            asio::async_write(client->socket, asio::buffer(*data), [client, data](std::error_code ec, std::size_t) {
                client->writing = false;
                if (ec) {
                    std::error_code ignored;
                    client->socket.close(ignored);
                }
            });
        }
        ++it;
    }
}

}  // namespace emu
