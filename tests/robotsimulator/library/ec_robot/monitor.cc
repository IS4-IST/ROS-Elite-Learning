#include "ec_robot/monitor.hpp"

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

Monitor::Monitor(std::string ip, std::uint16_t port, bool strict)
    : ip_(std::move(ip)), port_(port), strict_(strict), socket_(io_) {}

Monitor::~Monitor() { stop(); }

void Monitor::start(std::chrono::milliseconds wait) {
    socket_.connect(asio::ip::tcp::endpoint(asio::ip::make_address(ip_), port_));
    stopping_ = false;
    thread_ = std::thread([this] { run(); });
    if (wait.count() > 0 && !wait_for([](const RobotStatus&) { return true; }, wait)) {
        std::string err = error();
        stop();
        throw ECError("no valid monitor packet from " + ip_ + ":" + std::to_string(port_) +
                      (err.empty() ? "" : " (" + err + ")"));
    }
}

void Monitor::stop() {
    stopping_ = true;
    std::error_code ignored;
    socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);   // unblocks the reader
    socket_.close(ignored);
    if (thread_.joinable()) thread_.join();
}

RobotStatus Monitor::latest(std::chrono::milliseconds max_age) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_) throw ECError("no monitor data yet");
    if (max_age.count() > 0 && Clock::now() - latest_->received > max_age) {
        throw ECError("monitor data is stale (connection lost?)" + (error_.empty() ? "" : ": " + error_));
    }
    return *latest_;
}

std::optional<RobotStatus> Monitor::wait_for(const std::function<bool(const RobotStatus&)>& pred,
                                             std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = Clock::now() + timeout;
    while (true) {
        if (latest_ && pred(*latest_)) return latest_;
        if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
            if (latest_ && pred(*latest_)) return latest_;
            return std::nullopt;
        }
    }
}

void Monitor::add_callback(std::function<void(const RobotStatus&)> fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(std::move(fn));
}

std::string Monitor::error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}

void Monitor::run() {
    std::vector<std::uint8_t> packet;
    int consecutive_invalid = 0;
    try {
        while (!stopping_) {
            std::array<std::uint8_t, 4> head{};
            asio::read(socket_, asio::buffer(head));
            const std::uint32_t size = (std::uint32_t(head[0]) << 24) | (std::uint32_t(head[1]) << 16) |
                                       (std::uint32_t(head[2]) << 8) | std::uint32_t(head[3]);
            if (size <= 4 || size > 65536) throw std::runtime_error("implausible packet size " + std::to_string(size));
            packet.resize(size);
            std::memcpy(packet.data(), head.data(), 4);
            asio::read(socket_, asio::buffer(packet.data() + 4, size - 4));

            RobotStatus status = decode_packet(packet);
            ++packets_;
            if (strict_ && !status.valid) {
                ++invalid_packets_;
                if (++consecutive_invalid == 50) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    error_ = "50 packets in a row without the 0xEC8056EC matching word; "
                             "the layout may differ from the manual (use strict=false to inspect)";
                }
                continue;
            }
            consecutive_invalid = 0;
            std::vector<std::function<void(const RobotStatus&)>> callbacks;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ = status;
                callbacks = callbacks_;
            }
            cv_.notify_all();
            for (auto& fn : callbacks) fn(status);
        }
    } catch (const std::exception& e) {
        if (!stopping_) {
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = e.what();
        }
    }
}

}  // namespace ec
