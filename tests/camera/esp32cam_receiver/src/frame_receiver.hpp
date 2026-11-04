// Receiver for the ESP32-CAM UDP stream.
//
// Protocol:
//   - Open a TCP connection to the ESP32 (default port 3333). While it is open, the ESP32
//     streams every camera frame over UDP to this machine, port 3334. Closing it stops the stream.
//   - Each UDP packet: 12-byte little-endian header + up to 1400 bytes of JPEG data
//         uint32 frame_id, uint16 packet_index, uint16 packet_count, uint32 frame_size
//   - A frame with a missing packet is dropped; only the newest complete frame matters.
#pragma once

#include <asio.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace esp32cam {

struct Frame {
    uint32_t id = 0;
    std::vector<uint8_t> jpeg;                          // complete JPEG file contents
    std::chrono::steady_clock::time_point received;     // when the last packet arrived
};

struct Stats {
    uint64_t completed = 0;    // frames delivered
    uint64_t dropped = 0;      // frames abandoned because a packet never arrived
    uint64_t bad_packets = 0;  // packets that did not match the protocol
};

// Rebuilds frames from packets. Pure logic, no networking, so it can be tested on its own.
class FrameAssembler {
public:
    static constexpr size_t   kHeaderSize   = 12;
    static constexpr size_t   kPayload      = 1400;
    static constexpr uint32_t kMaxFrameSize = 4u * 1024 * 1024;  // sanity limit
    static constexpr size_t   kMaxPartial   = 8;                 // incomplete frames kept at most

    // Feed one UDP datagram. Returns true and fills `out` when a frame becomes complete.
    bool add(const uint8_t* data, size_t len, Frame& out);

    // Forget everything (e.g. after reconnecting: the ESP32 restarts frame ids at 0).
    void reset();

    const Stats& stats() const { return stats_; }

private:
    struct Partial {
        std::vector<uint8_t> data;
        std::vector<bool> got;
        uint16_t count = 0;
        uint16_t received = 0;
    };
    std::map<uint32_t, Partial> partial_;   // ordered by frame id: begin() is the oldest
    bool have_last_ = false;
    uint32_t last_complete_ = 0;
    Stats stats_;
};

// Connects to the ESP32, receives UDP packets and calls `on_frame` for every complete frame.
// All work happens on the io_context's thread; `on_frame` is called from that thread.
class FrameReceiver {
public:
    using Callback = std::function<void(Frame&&)>;

    FrameReceiver(asio::io_context& io, std::string host, uint16_t tcp_port, uint16_t udp_port,
                  Callback on_frame);

    void start();   // thread-safe: posts the work to the io_context
    void stop();    // thread-safe

    bool connected() const { return connected_.load(); }
    Stats stats() const { return {completed_.load(), dropped_.load(), bad_.load()}; }  // thread-safe

private:
    void connect();
    void watch_connection();
    void schedule_reconnect();
    void receive();

    asio::io_context& io_;
    std::string host_;
    uint16_t tcp_port_;
    uint16_t udp_port_;
    Callback on_frame_;

    asio::ip::address server_addr_;
    asio::ip::tcp::socket tcp_;
    asio::ip::udp::socket udp_;
    asio::steady_timer reconnect_timer_;
    asio::ip::udp::endpoint sender_;
    std::array<uint8_t, 2048> rx_{};
    std::array<char, 64> discard_{};
    bool stopping_ = false;

    FrameAssembler assembler_;
    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> completed_{0}, dropped_{0}, bad_{0};
};

}  // namespace esp32cam
