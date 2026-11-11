#include "monitor.hpp"
 
#include <cstring>
 
namespace ec {
namespace {
 
// Big-endian reader over a packet, bounded so it never reads the matching word
class Reader {
public:
    Reader(const std::vector<std::uint8_t>& p, std::size_t limit) : p_(p), limit_(limit) {}
 
    template <typename T>
    bool get(T& out) {
        if (pos_ + sizeof(T) > limit_) return false;
        std::uint64_t raw = 0;
        for (std::size_t i = 0; i < sizeof(T); ++i) raw = (raw << 8) | p_[pos_ + i];
        pos_ += sizeof(T);
        if constexpr (std::is_same_v<T, double>) {
            std::uint64_t bits = raw;
            std::memcpy(&out, &bits, sizeof(double));
        } else {
            out = static_cast<T>(raw);
        }
        return true;
    }
 
    template <typename T, std::size_t N>
    bool get(std::array<T, N>& out) {
        for (auto& v : out) {
            if (!get(v)) return false;
        }
        return true;
    }
 
private:
    const std::vector<std::uint8_t>& p_;
    std::size_t limit_;
    std::size_t pos_ = 0;
};
 
}  // namespace

// Connect `socket` to ip:port, giving up after `timeout`. No exceptions: refused, unreachable,
// timed_out (the timer won) or invalid_argument (not an IP address) come back as the error code.
inline asio::awaitable<std::error_code> connect_with_timeout(asio::ip::tcp::socket& socket, const std::string& ip,
                                                             unsigned short port, std::chrono::milliseconds timeout) {
    std::error_code ec;
    const asio::ip::tcp::endpoint endpoint(asio::ip::make_address(ip, ec), port);
    if (ec) co_return ec;
 
    // If the timer fires first it closes the socket, which ends the connect with operation_aborted.
    // (Asio 1.18 has no awaitable operators for racing two operations.)
    asio::steady_timer timer(socket.get_executor());
    timer.expires_after(timeout);
    timer.async_wait([&socket](std::error_code e) {
        if (!e) socket.close();
    });
    co_await socket.async_connect(endpoint, asio::redirect_error(asio::use_awaitable, ec));
    const bool timed_out = timer.expiry() <= asio::steady_timer::clock_type::now();
    timer.cancel();
    if (ec) {
        std::error_code ignored;
        socket.close(ignored);
        co_return (timed_out && ec == asio::error::operation_aborted) ? std::error_code(asio::error::timed_out) : ec;
    }
    socket.set_option(asio::ip::tcp::no_delay(true), ec);   // no Nagle delay on small messages
    co_return ec;
}

 
Joints RobotStatus::q() const {
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = joint_pos[i];
    return q;
}
 
Joints RobotStatus::qd() const {
    Joints v{};
    for (int i = 0; i < 6; ++i) v[i] = joint_speed[i];
    return v;
}
 
double RobotStatus::age_seconds() const {
    return std::chrono::duration<double>(Clock::now() - received).count();
}
 
RobotStatus decode_packet(const std::vector<std::uint8_t>& p) {
    RobotStatus s;
    s.received = Clock::now();
    if (p.size() >= 4) {
        std::uint32_t word = (std::uint32_t(p[p.size() - 4]) << 24) | (std::uint32_t(p[p.size() - 3]) << 16) |
                             (std::uint32_t(p[p.size() - 2]) << 8) | std::uint32_t(p[p.size() - 1]);
        s.valid = word == Monitor::kMatchingWord;
    }
    Reader r(p, p.size() >= 4 ? p.size() - 4 : 0);
    std::int32_t state = 0, servo = 0, synced = 0, mode = 0;
    std::uint8_t collision = 0, estop = 0, drag = 0, sensor = 0;
    // field order: manual table 3.1; stop at the first field that doesn't fit
    r.get(s.message_size) && r.get(s.timestamp_ms) && r.get(s.cycle_mode) && r.get(s.joint_pos) &&
        r.get(s.tcp_pose) && r.get(s.tcp_pose_user) && r.get(s.torque_permille) && r.get(state) &&
        r.get(servo) && r.get(synced) && r.get(s.motor_speed_rpm) && r.get(mode) && r.get(s.analog_in) &&
        r.get(s.analog_out) && r.get(s.digital_in) && r.get(s.digital_out) && r.get(collision) &&
        r.get(s.flange_pose) && r.get(s.flange_pose_user) && r.get(estop) && r.get(s.tcp_speed) &&
        r.get(s.joint_speed) && r.get(s.tcp_acc) && r.get(s.joint_acc) && r.get(s.joint_temperature) &&
        r.get(s.joint_torque) && r.get(s.ext_joint_torque) && r.get(s.ext_tcp_force_tool) && r.get(drag) &&
        r.get(sensor);
    s.robot_state = static_cast<State>(state);
    s.servo_ready = servo != 0;
    s.motor_synced = synced != 0;
    s.robot_mode = static_cast<Mode>(mode);
    s.collision = collision != 0;
    s.estop = estop != 0;
    s.drag_enabled = drag != 0;
    s.force_sensor_connected = sensor != 0;
    return s;
}
 
// --- error category ---------------------------------------------------------------------
namespace {
class MonitorCategory : public std::error_category {
public:
    const char* name() const noexcept override { return "ec-monitor"; }
    std::string message(int code) const override {
        switch (static_cast<MonitorErrc>(code)) {
            case MonitorErrc::bad_packet_size: return "implausible packet size: not an EC monitor stream";
            case MonitorErrc::no_matching_word:
                return "50 packets in a row without the 0xEC8056EC matching word: the layout may differ "
                       "from the manual (strict=false to inspect)";
            case MonitorErrc::no_packets: return "connected, but no monitor packet arrived in time";
        }
        return "unknown monitor error";
    }
};
}  // namespace
 
std::error_code make_error_code(MonitorErrc e) {
    static MonitorCategory category;
    return {static_cast<int>(e), category};
}
 
// --- Monitor ----------------------------------------------------------------------------------
Monitor::Monitor(asio::io_context& context, std::string ip, std::uint16_t port, bool strict,
                 std::chrono::milliseconds timeout)
    : context_(context), ip_(std::move(ip)), port_(port), strict_(strict), timeout_(timeout), socket_(context) {}
 
Monitor::~Monitor() {
    std::error_code ignored;
    socket_.close(ignored);   // directly: we're on the context's thread, or it has stopped
}
 
asio::awaitable<std::error_code> Monitor::connect() {
    if (std::error_code ec = co_await connect_with_timeout(socket_, ip_, port_, timeout_)) co_return ec;
    connected_ = true;
    asio::co_spawn(context_, reader(), asio::detached);   // one reader for the connection's life
 
    // Connected is not enough: 8055 (or any other service) also accepts connections.
    // Only a valid packet proves this is the monitor stream.
    // (no structured bindings on co_await results: GCC 11-13 have coroutine bugs there)
    const auto first = co_await wait_until([](const RobotStatus&) { return true; }, timeout_);
    if (first.first) {
        close();
        co_return first.first == asio::error::timed_out ? make_error_code(MonitorErrc::no_packets) : first.first;
    }
    co_return std::error_code{};
}
 
void Monitor::close() {
    asio::post(context_, [this] {
        std::error_code ignored;
        socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
        socket_.close(ignored);   // the reader's pending read ends: it wakes the waiters
        connected_ = false;
    });
}
 
std::optional<RobotStatus> Monitor::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}
 
std::error_code Monitor::last_error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}
 
void Monitor::wake_all(std::error_code ec) {
    for (Waiter* w : waiters_) {
        w->done = true;
        w->ec = ec;
        w->timer.cancel();
    }
}
 
asio::awaitable<std::pair<std::error_code, RobotStatus>>
Monitor::wait_until(std::function<bool(const RobotStatus&)> pred, std::chrono::milliseconds timeout) {
    if (!connected_) co_return std::pair{std::error_code(asio::error::not_connected), RobotStatus{}};
    Waiter w(context_);
    w.pred = std::move(pred);
    w.timer.expires_after(timeout);
    const auto it = waiters_.insert(waiters_.end(), &w);
    std::error_code ignored;
    co_await w.timer.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
    waiters_.erase(it);               // before the frame (and w) goes away
    if (!w.done) co_return std::pair{std::error_code(asio::error::timed_out), RobotStatus{}};
    co_return std::pair{w.ec, std::move(w.status)};
}
 
asio::awaitable<void> Monitor::reader() {
    std::vector<std::uint8_t> packet;
    int consecutive_invalid = 0;
    auto stop = [this](std::error_code ec) {
        connected_ = false;
        if (ec != asio::error::operation_aborted) {   // aborted = our own close(): not an error
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = ec;
        }
        wake_all(asio::error::not_connected);
    };
 
    for (;;) {
        // size field first (4 bytes, big-endian), then the rest of the packet
        std::array<std::uint8_t, 4> head{};
        std::error_code ec;
        co_await asio::async_read(socket_, asio::buffer(head), asio::redirect_error(asio::use_awaitable, ec));
        if (ec) { stop(ec); co_return; }
        const std::uint32_t size = (std::uint32_t(head[0]) << 24) | (std::uint32_t(head[1]) << 16) |
                                   (std::uint32_t(head[2]) << 8) | std::uint32_t(head[3]);
        if (size <= 4 || size > 65536) {
            std::error_code ignored;
            socket_.close(ignored);
            stop(make_error_code(MonitorErrc::bad_packet_size));
            co_return;
        }
        packet.resize(size);
        std::memcpy(packet.data(), head.data(), 4);
        co_await asio::async_read(socket_, asio::buffer(packet.data() + 4, size - 4),
                                  asio::redirect_error(asio::use_awaitable, ec));
        if (ec) { stop(ec); co_return; }
 
        RobotStatus status = decode_packet(packet);
        ++packets_;
        if (strict_ && !status.valid) {
            ++invalid_packets_;
            if (++consecutive_invalid == 50) {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = make_error_code(MonitorErrc::no_matching_word);   // a warning: keep reading
            }
            continue;
        }
        consecutive_invalid = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = status;
        }
        // wake every waiter whose condition this packet meets
        for (Waiter* w : waiters_) {
            if (!w->done && w->pred(status)) {
                w->done = true;
                w->status = status;
                w->timer.cancel();
            }
        }
    }
}
 
}  // namespace ec
