// Serves one camera's H.264 stream to any number of TCP clients, on the caller's io_context.
//
//   camera::Server server(io);
//   if (auto ec = server.listen("0.0.0.0", 5000)) { ... server.error_detail() ... }
//   // per encoded frame, on io's thread:
//   bool idr = server.wants_keyframe();       // ask the encoder for an IDR when true
//   server.broadcast(std::make_shared<const camera::EncodedFrame>(std::move(frame)));
//
// Each client starts at a keyframe. A client that is still sending the previous frame
// skips this one; since the next P-frames would depend on it, it then waits for the next
// keyframe, and wants_keyframe() asks for one so it doesn't wait a whole GOP.
// Latency stays at one frame: nothing queues behind a slow client.
#pragma once

#include <utility>   // before asio: Asio 1.18 uses std::exchange without including it
#include <asio.hpp>

#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "camera/encoder.h"

namespace camera {

class Server {
public:
    explicit Server(asio::io_context& io);
    ~Server();
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;

    std::error_code listen(const std::string& bind, unsigned short port);
    void close();

    void broadcast(std::shared_ptr<const EncodedFrame> frame);
    bool wants_keyframe();   // true once per request (a client joined or skipped a frame)

    std::size_t clients() const { return clients_.size(); }
    unsigned short port() const { return port_; }
    const std::string& error_detail() const { return detail_; }

private:
    struct Client {
        explicit Client(asio::ip::tcp::socket s) : socket(std::move(s)) {}
        asio::ip::tcp::socket socket;
        bool writing = false;
        bool needs_keyframe = true;
    };
    void accept();

    asio::ip::tcp::acceptor acceptor_;
    std::vector<std::shared_ptr<Client>> clients_;
    bool want_keyframe_ = false;
    unsigned short port_ = 0;
    std::string detail_;
};

}  // namespace camera
