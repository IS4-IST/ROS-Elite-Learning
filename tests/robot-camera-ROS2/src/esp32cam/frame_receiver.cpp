#include "frame_receiver.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace esp32cam {

namespace {
uint16_t rd16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
}  // namespace

// ============================ FrameAssembler ================================

void FrameAssembler::reset()
{
    partial_.clear();
    have_last_ = false;
}

bool FrameAssembler::add(const uint8_t* data, size_t len, Frame& out)
{
    if (len < kHeaderSize) { stats_.bad_packets++; return false; }

    const uint32_t id    = rd32(data);
    const uint16_t index = rd16(data + 4);
    const uint16_t count = rd16(data + 6);
    const uint32_t size  = rd32(data + 8);
    const uint8_t* payload = data + kHeaderSize;
    const size_t   payload_len = len - kHeaderSize;

    // Validate against the protocol before touching any buffer
    if (size == 0 || size > kMaxFrameSize || count == 0 || index >= count ||
        count != (size + kPayload - 1) / kPayload) {
        stats_.bad_packets++;
        return false;
    }
    const size_t offset = size_t(index) * kPayload;
    const size_t expected = std::min(kPayload, size_t(size) - offset);
    if (payload_len != expected) { stats_.bad_packets++; return false; }

    // Packets of frames older than the last one shown are useless
    if (have_last_ && id <= last_complete_) {
        if (last_complete_ - id < 1000) return false;      // simply late
        reset();                                           // far behind: the sender restarted
    }

    auto [it, inserted] = partial_.try_emplace(id);
    Partial& p = it->second;
    if (inserted) {
        p.data.resize(size);
        p.got.assign(count, false);
        p.count = count;
    } else if (p.count != count || p.data.size() != size) {
        stats_.bad_packets++;                              // inconsistent with earlier packets
        return false;
    }

    if (!p.got[index]) {
        std::memcpy(p.data.data() + offset, payload, payload_len);
        p.got[index] = true;
        p.received++;
    }

    if (p.received < p.count) {
        // Bound memory: if too many frames are incomplete, give up on the oldest
        while (partial_.size() > kMaxPartial) {
            partial_.erase(partial_.begin());
            stats_.dropped++;
        }
        return false;
    }

    // Complete. Older incomplete frames will never be shown: drop them.
    out.id = id;
    out.jpeg = std::move(p.data);
    out.received = std::chrono::steady_clock::now();
    while (partial_.begin()->first != id) {
        partial_.erase(partial_.begin());
        stats_.dropped++;
    }
    partial_.erase(partial_.begin());
    last_complete_ = id;
    have_last_ = true;
    stats_.completed++;
    return true;
}

// ============================ FrameReceiver =================================

FrameReceiver::FrameReceiver(asio::io_context& io, std::string host, uint16_t tcp_port,
                             uint16_t udp_port, Callback on_frame)
    : io_(io), host_(std::move(host)), tcp_port_(tcp_port), udp_port_(udp_port),
      on_frame_(std::move(on_frame)), tcp_(io), udp_(io), reconnect_timer_(io)
{
}

void FrameReceiver::start()
{
    asio::post(io_, [this] {
        std::error_code ec;
        server_addr_ = asio::ip::make_address(host_, ec);
        if (ec) {
            std::fprintf(stderr, "invalid ESP32 address '%s': %s\n", host_.c_str(), ec.message().c_str());
            return;
        }

        using asio::ip::udp;
        udp_.open(udp::v4(), ec);
        if (!ec) udp_.set_option(asio::socket_base::receive_buffer_size(4 * 1024 * 1024), ec);
        if (!ec) udp_.bind(udp::endpoint(udp::v4(), udp_port_), ec);
        if (ec) {
            std::fprintf(stderr, "cannot listen on UDP port %u: %s\n", udp_port_, ec.message().c_str());
            return;
        }
        receive();
        connect();
    });
}

void FrameReceiver::stop()
{
    asio::post(io_, [this] {
        stopping_ = true;
        std::error_code ignored;
        reconnect_timer_.cancel();
        tcp_.close(ignored);
        udp_.close(ignored);
        connected_ = false;
    });
}

void FrameReceiver::connect()
{
    if (stopping_) return;
    tcp_ = asio::ip::tcp::socket(io_);
    tcp_.async_connect({server_addr_, tcp_port_}, [this](std::error_code ec) {
        if (ec) {
            if (ec != asio::error::operation_aborted) schedule_reconnect();
            return;
        }
        std::error_code ignored;
        tcp_.set_option(asio::ip::tcp::no_delay(true), ignored);
        assembler_.reset();            // the ESP32 restarts frame ids at 0 for each client
        connected_ = true;
        std::fprintf(stderr, "connected to %s:%u, receiving on UDP %u\n",
                     host_.c_str(), tcp_port_, udp_port_);
        watch_connection();
    });
}

// The ESP32 sends nothing over TCP; a pending read just tells us when it closes.
void FrameReceiver::watch_connection()
{
    tcp_.async_read_some(asio::buffer(discard_), [this](std::error_code ec, std::size_t) {
        if (!ec) { watch_connection(); return; }
        if (ec == asio::error::operation_aborted) return;
        connected_ = false;
        std::error_code ignored;
        tcp_.close(ignored);
        std::fprintf(stderr, "connection lost (%s), reconnecting...\n", ec.message().c_str());
        schedule_reconnect();
    });
}

void FrameReceiver::schedule_reconnect()
{
    if (stopping_) return;
    reconnect_timer_.expires_after(std::chrono::seconds(1));
    reconnect_timer_.async_wait([this](std::error_code ec) {
        if (!ec) connect();
    });
}

void FrameReceiver::receive()
{
    udp_.async_receive_from(asio::buffer(rx_), sender_, [this](std::error_code ec, std::size_t n) {
        if (ec == asio::error::operation_aborted) return;
        if (!ec && sender_.address() == server_addr_) {     // ignore packets from anyone else
            Frame frame;
            if (assembler_.add(rx_.data(), n, frame)) on_frame_(std::move(frame));
            const Stats& s = assembler_.stats();
            completed_ = s.completed;
            dropped_ = s.dropped;
            bad_ = s.bad_packets;
        }
        receive();
    });
}

}  // namespace esp32cam
