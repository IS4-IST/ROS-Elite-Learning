// Event-driven RTSI reader for Elite CS robots (Elite CS SDK).
//
// Uses the SDK's low-level RtsiClientInterface: receiveData() blocks until the next RTSI
// packet arrives, so every sample is handed to the callback as soon as it is received,
// without polling. Reconnects automatically when the robot is unreachable or the link drops.
#pragma once

#include <Elite/DataType.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace ELITE { class RtsiClientInterface; class RtsiRecipe; }

namespace robot {

struct RobotSample {
    double controller_time = 0.0;                        // RTSI "timestamp": controller clock [s]
    std::chrono::steady_clock::time_point received;      // PC clock, when the packet was parsed

    ELITE::vector6d_t q{};          // actual joint positions [rad]
    ELITE::vector6d_t qd{};         // actual joint velocities [rad/s]
    ELITE::vector6d_t tau{};        // actual joint torques [Nm]
    ELITE::vector6d_t q_target{};   // target joint positions [rad]
    ELITE::vector6d_t tcp_pose{};   // actual TCP pose [x y z rx ry rz]: metres + rotation vector [rad]
    ELITE::vector6d_t tcp_speed{};  // actual TCP velocity [vx vy vz wx wy wz]
    ELITE::vector6d_t tcp_force{};  // actual TCP force/torque [fx fy fz tx ty tz]
    int32_t robot_mode = -2;        // ELITE::RobotMode
    int32_t safety_mode = -2;       // ELITE::SafetyMode
    double speed_scaling = 0.0;
};

class RtsiReader {
public:
    using SampleCallback = std::function<void(const RobotSample&)>;
    using StatusCallback = std::function<void(bool connected, const std::string& message)>;

    RtsiReader(std::string robot_ip, double frequency_hz, SampleCallback on_sample,
               StatusCallback on_status = nullptr);
    ~RtsiReader();

    RtsiReader(const RtsiReader&) = delete;
    RtsiReader& operator=(const RtsiReader&) = delete;

    void start();
    void stop();     // returns within ~1 s (the SDK's receive timeout)
    bool connected() const { return connected_.load(); }

private:
    void run();
    bool connect_once();
    void disconnect();

    std::string ip_;
    double frequency_;
    SampleCallback on_sample_;
    StatusCallback on_status_;

    std::unique_ptr<ELITE::RtsiClientInterface> client_;
    std::shared_ptr<ELITE::RtsiRecipe> recipe_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
};

const char* robot_mode_name(int32_t mode);
const char* safety_mode_name(int32_t mode);

}  // namespace robot
