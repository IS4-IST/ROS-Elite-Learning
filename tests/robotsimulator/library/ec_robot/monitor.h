// Async reader for the EC controller's monitor port 8056 (sketch).
//
// The controller pushes a 1024-byte big-endian status packet every ~8 ms. One reader
// coroutine on your io_context decodes them and keeps the latest. Expected failures come
// back as std::error_code, never as exceptions.
//
// Threads: connect/close/wait_until run on the io_context's thread. latest() may be called
// from any thread (the GUI, ROS): it is the one place with a lock, a copy under a mutex.
#pragma once

#include <utility>   // before asio.hpp: Asio 1.18.1 uses std::exchange without including it
#include "asio.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "types.hpp"   // Joints, Pose, Mode, State, Clock

namespace ec {

struct RobotStatus {
    Clock::time_point received{};     // local receive time
    bool valid = false;               // matching word present

    std::uint32_t message_size = 0;
    std::uint64_t timestamp_ms = 0;
    std::uint8_t cycle_mode = 0;
    std::array<double, 8> joint_pos{};          // deg (first 6 = arm)
    Pose tcp_pose{};                            // base frame, mm / rad
    Pose tcp_pose_user{};
    std::array<double, 8> torque_permille{};
    State robot_state = State::Stop;
    bool servo_ready = false;
    bool motor_synced = false;                  // can_motor_run
    std::array<std::int32_t, 8> motor_speed_rpm{};
    Mode robot_mode = Mode::Teach;
    std::array<double, 3> analog_in{};
    std::array<double, 5> analog_out{};
    std::uint64_t digital_in = 0;
    std::uint64_t digital_out = 0;
    bool collision = false;
    Pose flange_pose{};
    Pose flange_pose_user{};
    bool estop = false;
    double tcp_speed = 0;                       // mm/s
    std::array<double, 8> joint_speed{};        // deg/s
    double tcp_acc = 0;
    std::array<double, 8> joint_acc{};
    std::array<double, 6> joint_temperature{};  // deg C
    std::array<double, 6> joint_torque{};       // Nm
    std::array<double, 6> ext_joint_torque{};   // Nm
    std::array<double, 6> ext_tcp_force_tool{}; // N / Nm
    bool drag_enabled = false;
    bool force_sensor_connected = false;

    Joints q() const;            // 6 arm joints, deg
    Joints qd() const;           // 6 arm joint speeds, deg/s
    double age_seconds() const;  // time since received
};

RobotStatus decode_packet(const std::vector<std::uint8_t>& packet);

// The monitor's own failures, beside the usual asio ones (refused, timed_out, ...)
enum class MonitorErrc {
    bad_packet_size = 1,   // the size field is implausible: not an EC monitor stream
    no_matching_word,      // 50 packets in a row without 0xEC8056EC: layout differs from the manual
    no_packets,            // connected, but nothing arrived within the timeout
};
std::error_code make_error_code(MonitorErrc e);

class Monitor {
public:
    static constexpr std::uint16_t kPort = 8056;
    static constexpr std::uint32_t kMatchingWord = 0xEC8056ECu;

    Monitor(asio::io_context& context, std::string ip, std::uint16_t port = kPort, bool strict = true,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;

    // Connect, start the reader and wait for the first valid packet (all within the timeout).
    asio::awaitable<std::error_code> connect();
    void close();                          // wakes every waiter with not_connected
    bool connected() const { return connected_; }

    // Latest valid packet, if any, from any thread. No age check: look at age_seconds().
    std::optional<RobotStatus> latest() const;

    // Wait for the next packet for which pred(status) is true, or time out.
    // Replaces polling loops with sleeps, e.g. "until state is Stop" or "until servos are on".
    asio::awaitable<std::pair<std::error_code, RobotStatus>>
    wait_until(std::function<bool(const RobotStatus&)> pred, std::chrono::milliseconds timeout);

    std::uint64_t packets() const { return packets_; }
    std::uint64_t invalid_packets() const { return invalid_packets_; }
    std::error_code last_error() const;   // why the reader stopped, or a layout warning

private:
    struct Waiter {
        explicit Waiter(asio::io_context& context) : timer(context) {}
        std::function<bool(const RobotStatus&)> pred;
        asio::steady_timer timer;   // expires = timeout; cancelled by the reader = condition met or closed
        bool done = false;
        std::error_code ec;
        RobotStatus status;
    };

    asio::awaitable<void> reader();
    void wake_all(std::error_code ec);

    asio::io_context& context_;
    std::string ip_;
    std::uint16_t port_;
    bool strict_;
    std::chrono::milliseconds timeout_;
    asio::ip::tcp::socket socket_;
    std::atomic<bool> connected_{false};

    std::list<Waiter*> waiters_;          // only touched on the io_context's thread

    mutable std::mutex mutex_;            // guards latest_ and error_ for readers on other threads
    std::optional<RobotStatus> latest_;
    std::error_code error_;
    std::atomic<std::uint64_t> packets_{0}, invalid_packets_{0};
};

}  // namespace ec

template <>
struct std::is_error_code_enum<ec::MonitorErrc> : std::true_type {};
