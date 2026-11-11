// Receives one camera's H.264 stream (simulator or ESP32-P4) on the caller's io_context,
// decodes it and hands every frame to a callback. Reconnects on its own.
//
//   camera::Client cam(io, "127.0.0.1", 5000);
//   cam.on_frame([](const camera::DecodedFrame& image, const camera::FrameStamp& stamp) { ... });
//   cam.start();
//   ... cam.stats() ...      // on io's thread
//   cam.close();
//
// A frame is decoded as soon as its last byte arrives: the timestamp SEI says how many
// bytes follow it (stream_format.h). The callback runs on io's thread, so with the GUI's
// io_context it can upload to an OpenGL texture directly.
// Destroy the client before its io_context.
#pragma once

#include <utility>   // before asio: Asio 1.18 uses std::exchange without including it
#include <asio.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <system_error>
#include <vector>

#include "camera/decoder.h"
#include "camera/stream_format.h"

namespace camera {

struct ClientStats {
    bool connected = false;
    int width = 0, height = 0;
    double fps = 0;             // decoded frames per second, over the last second
    double kbps = 0;            // received kbit/s, over the last second
    double decode_ms = 0;       // smoothed decode + colour conversion time
    double latency_ms = 0;      // smoothed arrival - capture (includes the clocks' offset)
    std::uint64_t frames = 0;   // decoded since start
    std::uint64_t skipped = 0;  // frames the camera never sent us (gaps in frame_id)
    std::uint64_t errors = 0;   // frames that failed to decode
    std::chrono::steady_clock::time_point last_frame{};   // when the latest frame was decoded
};

class Client {
public:
    using FrameCallback = std::function<void(const DecodedFrame&, const FrameStamp&)>;

    Client(asio::io_context& io, std::string host, unsigned short port);
    ~Client();
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    void on_frame(FrameCallback callback) { callback_ = std::move(callback); }
    void start();
    void close();

    const ClientStats& stats();   // refreshes fps and kbit/s over the last second
    const std::string& status() const { return status_; }   // "connecting", "receiving", a reason
    const std::string& host() const { return host_; }
    unsigned short port() const { return port_; }

private:
    asio::awaitable<void> run();
    asio::awaitable<std::error_code> receive();
    void consume();   // cut complete frames out of buffer_ and decode them
    void decode_frame(std::span<const std::uint8_t> access_unit, const FrameStamp& stamp);
    void update_rates();

    asio::io_context& io_;
    std::string host_;
    unsigned short port_;
    asio::ip::tcp::socket socket_;
    asio::steady_timer retry_;
    bool running_ = false;
    bool closing_ = false;

    Decoder decoder_;
    DecodedFrame image_;
    FrameCallback callback_;

    std::vector<std::uint8_t> buffer_;   // received bytes not yet decoded
    std::size_t scan_ = 0;               // where to look for the next start code
    bool have_stamp_ = false;            // a stamp SEI was found; waiting for its `follow` bytes
    FrameStamp stamp_;
    std::size_t body_ = 0;               // where the bytes after that SEI start
    bool have_last_id_ = false;
    std::uint64_t last_id_ = 0;

    ClientStats stats_;
    std::string status_ = "not started";
    std::chrono::steady_clock::time_point window_start_;
    std::uint64_t window_frames_ = 0, window_bytes_ = 0;
};

}  // namespace camera
