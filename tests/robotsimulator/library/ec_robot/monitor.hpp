// Reader for the Elite EC monitor interface, port 8056.
//
// Reference: EC SDK-Socket Script Manual Ver3.19.2, chapter 3.
//  - one big-endian packet roughly every 8 ms
//  - 1024 bytes: 801 bytes of fields, reserved space, and a 4-byte matching
//    word that must equal 0xEC8056EC, otherwise the packet is invalid
//  - packets pile up if not read promptly, so a thread reads them continuously
//    and only the newest one is kept
// Units: joints deg, poses mm + rad, joint speed deg/s, torque Nm.
// Verified on controller V3.11.2: 0 invalid packets; temperatures and torques
// read -40 C / 0 while the servos are off.
#pragma once

#include <asio.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ec_robot/types.hpp"

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

// Decode one complete packet (exposed for tests). Fields that don't fit are left at defaults.
RobotStatus decode_packet(const std::vector<std::uint8_t>& packet);

class Monitor {
public:
    static constexpr std::uint16_t kPort = 8056;
    static constexpr std::uint32_t kMatchingWord = 0xEC8056ECu;

    explicit Monitor(std::string ip, std::uint16_t port = kPort, bool strict = true);
    ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;

    // Connect and start the reader thread; waits up to `wait` for the first valid packet.
    void start(std::chrono::milliseconds wait = std::chrono::milliseconds(2000));
    void stop();

    // Newest valid status; throws if none yet or older than max_age.
    RobotStatus latest(std::chrono::milliseconds max_age = std::chrono::milliseconds(100)) const;

    // Block until pred(status) holds; returns the status, or nullopt on timeout.
    std::optional<RobotStatus> wait_for(const std::function<bool(const RobotStatus&)>& pred,
                                        std::chrono::milliseconds timeout) const;

    // Called from the reader thread for every valid packet; keep it fast.
    void add_callback(std::function<void(const RobotStatus&)> fn);

    std::uint64_t packets() const { return packets_; }
    std::uint64_t invalid_packets() const { return invalid_packets_; }
    std::string error() const;

private:
    void run();

    std::string ip_;
    std::uint16_t port_;
    bool strict_;
    asio::io_context io_;
    asio::ip::tcp::socket socket_;
    std::thread thread_;
    std::atomic<bool> stopping_{false};

    mutable std::mutex mutex_;
    mutable std::condition_variable cv_;
    std::optional<RobotStatus> latest_;
    std::vector<std::function<void(const RobotStatus&)>> callbacks_;
    std::string error_;
    std::atomic<std::uint64_t> packets_{0};
    std::atomic<std::uint64_t> invalid_packets_{0};
};

}  // namespace ec
