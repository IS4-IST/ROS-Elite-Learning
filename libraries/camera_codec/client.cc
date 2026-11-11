#include "camera/client.h"

#include "ec_robot/net.h"   // connect_with_timeout (header-only)

namespace camera {

namespace net = ::ec;   // the robot library's helpers
using namespace std::chrono_literals;

namespace {
constexpr std::size_t kMaxBuffered = 8 << 20;   // no frame is this big: we lost sync, start over
constexpr auto kStallTimeout = 3s;              // no bytes for this long: reconnect
}  // namespace

Client::Client(asio::io_context& io, std::string host, unsigned short port)
    : io_(io), host_(std::move(host)), port_(port), socket_(io), retry_(io) {}

Client::~Client() { close(); }

void Client::start() {
    if (running_) return;
    running_ = true;
    closing_ = false;
    asio::co_spawn(io_, run(), asio::detached);
}

void Client::close() {
    closing_ = true;
    std::error_code ignored;
    socket_.close(ignored);
    retry_.cancel();
}

const ClientStats& Client::stats() {
    update_rates();
    return stats_;
}

asio::awaitable<void> Client::run() {
    while (!closing_) {
        status_ = "connecting to " + host_ + ":" + std::to_string(port_);
        const std::error_code ec = co_await receive();
        stats_.connected = false;
        if (closing_) break;
        status_ = "no stream (" + ec.message() + "), retrying";
        retry_.expires_after(1s);
        std::error_code ignored;
        co_await retry_.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
    }
    running_ = false;
}

asio::awaitable<std::error_code> Client::receive() {
    std::error_code ec = co_await net::connect_with_timeout(socket_, host_, port_, 2000ms);
    if (ec) co_return ec;
    if (closing_) co_return asio::error::operation_aborted;

    if (ec = decoder_.open(); ec) {   // a fresh decoder per connection: the stream restarts
        socket_.close(ec);
        co_return CameraErrc::open_failed;
    }
    buffer_.clear();
    scan_ = 0;
    have_stamp_ = false;
    have_last_id_ = false;
    stats_.connected = true;
    status_ = "waiting for the first frame";

    // the watchdog closes the socket when nothing arrives for a while, which ends the read
    asio::steady_timer watchdog(io_);
    std::vector<std::uint8_t> chunk(64 * 1024);
    for (;;) {
        watchdog.expires_after(kStallTimeout);
        watchdog.async_wait([this](std::error_code e) {
            if (!e) {
                std::error_code ignored;
                socket_.close(ignored);
            }
        });
        const std::size_t n =
            co_await socket_.async_read_some(asio::buffer(chunk), asio::redirect_error(asio::use_awaitable, ec));
        const bool stalled = watchdog.expiry() <= asio::steady_timer::clock_type::now();
        watchdog.cancel();
        if (ec) {
            std::error_code ignored;
            socket_.close(ignored);
            co_return (stalled && ec == asio::error::operation_aborted) ? std::error_code(asio::error::timed_out) : ec;
        }
        buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(n));
        window_bytes_ += n;
        consume();
    }
}

void Client::consume() {
    for (;;) {
        const std::span<const std::uint8_t> data(buffer_);
        if (!have_stamp_) {
            // look at the next complete NAL unit: is it our timestamp SEI?
            const std::size_t sc = find_start_code(data, scan_);
            if (sc == npos) break;
            const std::size_t next = find_start_code(data, sc + 3);
            if (next == npos) {   // not complete yet
                scan_ = sc;
                break;
            }
            std::size_t end = next;
            while (end > sc + 3 && data[end - 1] == 0) --end;   // leading 00 of a 4-byte start code
            if (auto stamp = parse_stamp_sei(Nal{data.subspan(sc + 3, end - sc - 3)})) {
                stamp_ = *stamp;
                body_ = end;
                have_stamp_ = true;
            } else {
                scan_ = next;   // SPS, PPS, other SEI: stay in the buffer, part of this frame
            }
            continue;
        }
        const std::size_t frame_end = body_ + stamp_.follow;
        if (data.size() < frame_end) break;   // the rest of the frame hasn't arrived
        decode_frame(data.first(frame_end), stamp_);
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_end));
        scan_ = 0;
        have_stamp_ = false;
    }
    if (buffer_.size() > kMaxBuffered) {   // a stream without our SEI, or corrupted: drop it
        buffer_.clear();
        scan_ = 0;
        have_stamp_ = false;
        status_ = "no timestamp SEI in the stream (is it a camera in our format?)";
    }
}

void Client::decode_frame(std::span<const std::uint8_t> access_unit, const FrameStamp& stamp) {
    const auto t0 = std::chrono::steady_clock::now();
    bool got = false;
    if (std::error_code ec = decoder_.decode(access_unit, image_, got)) {
        ++stats_.errors;
        status_ = ec.message() + ": " + decoder_.error_detail();
        return;
    }
    if (have_last_id_ && stamp.frame_id > last_id_ + 1) stats_.skipped += stamp.frame_id - last_id_ - 1;
    last_id_ = stamp.frame_id;
    have_last_id_ = true;
    if (!got) return;

    const double decode_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const std::int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
    const double latency_ms = (now_ns - stamp.capture_ns) / 1e6;
    const bool first = stats_.frames == 0;
    stats_.decode_ms = first ? decode_ms : 0.9 * stats_.decode_ms + 0.1 * decode_ms;
    stats_.latency_ms = first ? latency_ms : 0.9 * stats_.latency_ms + 0.1 * latency_ms;
    stats_.last_frame = std::chrono::steady_clock::now();
    stats_.width = image_.width;
    stats_.height = image_.height;
    ++stats_.frames;
    ++window_frames_;
    status_ = "receiving";
    if (callback_) callback_(image_, stamp);
}

void Client::update_rates() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - window_start_).count();
    if (elapsed < 1.0) return;
    stats_.fps = window_frames_ / elapsed;
    stats_.kbps = window_bytes_ * 8 / 1000.0 / elapsed;
    window_frames_ = window_bytes_ = 0;
    window_start_ = now;
}

}  // namespace camera
