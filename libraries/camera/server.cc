#include "camera/server.h"

namespace camera {

Server::Server(asio::io_context& io) : acceptor_(io) {}

Server::~Server() { close(); }

std::error_code Server::listen(const std::string& bind, unsigned short port) {
    std::error_code ec;
    const asio::ip::address address = asio::ip::make_address(bind, ec);
    if (ec) {
        detail_ = "bad bind address " + bind;
        return CameraErrc::listen_failed;
    }
    const asio::ip::tcp::endpoint endpoint(address, port);
    acceptor_.open(endpoint.protocol(), ec);
    if (!ec) acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true), ec);
    if (!ec) acceptor_.bind(endpoint, ec);
    if (!ec) acceptor_.listen(asio::socket_base::max_listen_connections, ec);
    if (ec) {
        detail_ = bind + ":" + std::to_string(port) + ": " + ec.message();
        std::error_code ignored;
        acceptor_.close(ignored);
        return CameraErrc::listen_failed;
    }
    port_ = port;
    accept();
    return {};
}

void Server::close() {
    std::error_code ignored;
    acceptor_.close(ignored);
    for (auto& client : clients_) client->socket.close(ignored);
    clients_.clear();
}

void Server::accept() {
    acceptor_.async_accept([this](std::error_code ec, asio::ip::tcp::socket socket) {
        if (ec == asio::error::operation_aborted) return;   // closed
        if (!ec) {
            socket.set_option(asio::ip::tcp::no_delay(true), ec);
            clients_.push_back(std::make_shared<Client>(std::move(socket)));
            want_keyframe_ = true;   // so the newcomer doesn't wait a whole GOP
        }
        accept();
    });
}

bool Server::wants_keyframe() {
    const bool want = want_keyframe_;
    want_keyframe_ = false;
    return want;
}

void Server::broadcast(std::shared_ptr<const EncodedFrame> frame) {
    for (auto it = clients_.begin(); it != clients_.end();) {
        const std::shared_ptr<Client> client = *it;
        if (!client->socket.is_open()) {
            it = clients_.erase(it);
            continue;
        }
        ++it;
        if (client->needs_keyframe && !frame->keyframe) continue;   // can't decode from here
        if (client->writing) {                                      // still busy with the last frame
            client->needs_keyframe = true;
            want_keyframe_ = true;
            continue;
        }
        client->needs_keyframe = false;
        client->writing = true;
        asio::async_write(client->socket, asio::buffer(frame->bytes),
                          [client, frame](std::error_code ec, std::size_t) {   // frame stays alive while sending
                              client->writing = false;
                              if (ec) {
                                  std::error_code ignored;
                                  client->socket.close(ignored);   // removed on the next broadcast
                              }
                          });
    }
}

}  // namespace camera
