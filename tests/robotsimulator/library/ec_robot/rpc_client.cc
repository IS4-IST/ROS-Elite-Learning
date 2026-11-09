#include "ec_robot/rpc_client.hpp"

#include <istream>

namespace ec {

const char* to_string(Mode m) {
    switch (m) {
        case Mode::Teach: return "teach";
        case Mode::Play: return "play";
        case Mode::Remote: return "remote";
    }
    return "unknown";
}

const char* to_string(State s) {
    switch (s) {
        case State::Stop: return "stop";
        case State::Pause: return "pause";
        case State::EStop: return "emergency stop";
        case State::Running: return "running";
        case State::Alarm: return "alarm";
        case State::Collision: return "collision";
    }
    return "unknown";
}

nlohmann::json decode_result(const nlohmann::json& result) {
    nlohmann::json value = result;
    for (int i = 0; i < 3 && value.is_string(); ++i) {
        auto parsed = nlohmann::json::parse(value.get<std::string>(), nullptr, /*allow_exceptions=*/false);
        if (parsed.is_discarded()) break;   // a plain string, not JSON text
        value = std::move(parsed);
    }
    return value;
}

RpcClient::RpcClient(std::string ip, std::uint16_t port, std::chrono::milliseconds timeout)
    : ip_(std::move(ip)), port_(port), timeout_(timeout), socket_(io_) {}

RpcClient::~RpcClient() { close(); }

void RpcClient::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    asio::ip::tcp::endpoint endpoint(asio::ip::make_address(ip_), port_);
    std::error_code ec;
    bool done = false;
    socket_.async_connect(endpoint, [&](const std::error_code& e) { ec = e; done = true; });
    io_.restart();
    io_.run_for(timeout_);
    if (!done) {
        socket_.close();
        io_.restart();
        io_.run();
        throw ECError("connect to " + ip_ + ":" + std::to_string(port_) + " timed out");
    }
    if (ec) throw ECError("connect to " + ip_ + ":" + std::to_string(port_) + " failed: " + ec.message());
    socket_.set_option(asio::ip::tcp::no_delay(true));   // no Nagle delay on small messages
    buffer_.consume(buffer_.size());
    connected_ = true;
}

void RpcClient::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::error_code ignored;
    socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
    socket_.close(ignored);
    connected_ = false;
}

bool RpcClient::connected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_;
}

nlohmann::json RpcClient::call(const std::string& method, const nlohmann::json& params,
                               std::optional<std::chrono::milliseconds> timeout) {
    nlohmann::json reply;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto deadline = Clock::now() + timeout.value_or(timeout_);
        const auto id = send_locked(method, params);
        while (true) {
            reply = nlohmann::json::parse(read_line_locked(deadline, method));
            if (reply.value("id", std::uint64_t{0}) == id) break;
            // reply to an earlier request (e.g. one that timed out): discard it
        }
    }
    if (reply.contains("error")) {
        const auto& err = reply["error"];
        if (err.is_object()) {
            throw ECError(method + ": " + err.value("message", std::string("error")),
                          err.value("code", 0), method);
        }
        throw ECError(method + ": " + err.dump(), 0, method);
    }
    return decode_result(reply.value("result", nlohmann::json()));
}

void RpcClient::notify(const std::string& method, const nlohmann::json& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    send_locked(method, params);
}

std::uint64_t RpcClient::send_locked(const std::string& method, const nlohmann::json& params) {
    if (!connected_) throw ECError("not connected", 0, method);
    const auto id = ++next_id_;
    nlohmann::json msg = {{"jsonrpc", "2.0"}, {"method", method}, {"params", params}, {"id", id}};
    std::string text = msg.dump() + "\n";
    std::error_code ec;
    asio::write(socket_, asio::buffer(text), ec);
    if (ec) {
        connected_ = false;
        throw ECError(method + ": send failed: " + ec.message(), 0, method);
    }
    return id;
}

std::string RpcClient::read_line_locked(Clock::time_point deadline, const std::string& method) {
    // async read with a deadline: run the io_context only until the deadline
    std::error_code ec;
    std::size_t bytes = 0;
    bool done = false;
    asio::async_read_until(socket_, buffer_, '\n', [&](const std::error_code& e, std::size_t n) {
        ec = e;
        bytes = n;
        done = true;
    });
    io_.restart();
    const auto now = Clock::now();
    if (deadline > now) io_.run_for(deadline - now);
    if (!done) {
        socket_.cancel();          // abort the pending read and let its handler run
        io_.restart();
        io_.run();
        throw ECError(method + ": no reply within timeout", 0, method);
    }
    if (ec) {
        connected_ = false;
        throw ECError(method + ": connection lost: " + ec.message(), 0, method);
    }
    std::istream in(&buffer_);
    std::string line;
    std::getline(in, line);
    (void)bytes;
    if (line.find_first_not_of(" \r\t") == std::string::npos) return read_line_locked(deadline, method);
    return line;
}

}  // namespace ec
