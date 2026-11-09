// High-level interface to an Elite EC-series robot (EC SDK-Socket Manual Ver3.19.2).
//
// Units: joints deg, poses [x, y, z, rx, ry, rz] mm + rad (fixed-axis XYZ).
//
// Behaviour carried over from tests on controller V3.11.2:
//  - Cartesian targets are never sent as poses: moveByLine's `target_pose` form
//    moved the arm far from the target on V3.11.2. move_line() converts the pose
//    with the controller's IK, checks the joint change, and sends `targetPos`.
//  - The brakes must be released on the teach pendant before power_on() can sync
//    the encoders ("the brake of robot is not opened").
//  - The active tool (TCP + payload) is selected per tool number; call set_tool().
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ec_robot/monitor.hpp"
#include "ec_robot/rpc_client.hpp"
#include "ec_robot/streamer.hpp"
#include "ec_robot/types.hpp"

namespace ec {

struct ToolInfo {
    Pose tcp{};                 // offset from the flange, mm / rad
    double mass = 0;            // kg
    std::array<double, 3> cog{};// mm
};

class Robot {
public:
    explicit Robot(std::string ip = "192.168.0.11", bool strict_monitor = true);
    ~Robot();

    void connect();     // 8055 + 8056
    void close();

    RpcClient& rpc() { return rpc_; }
    Monitor& monitor() { return monitor_; }

    // --- state -----------------------------------------------------------
    RobotStatus status(std::chrono::milliseconds max_age = std::chrono::milliseconds(100)) const;
    std::string version();
    std::string describe() const;
    void require_remote() const;
    void require_ready() const;   // remote, servos on, synced, state stop

    // --- power / setup -----------------------------------------------------
    void power_on(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    void power_off();
    void set_speed(double percent);     // global override, 0.05..100 %
    void set_tool(int tool_num);        // TCP + payload for remote mode (2.2.2.37)
    ToolInfo tool_info(int tool_num);
    void stop();

    // --- point-to-point ------------------------------------------------------
    // Joint move, speed 0.01..100 %. Blocks until stopped when wait = true.
    void move_joint(const Joints& q, double speed = 10.0, bool wait = true,
                    std::chrono::seconds timeout = std::chrono::seconds(60));
    // IK solution for a straight-line move to pose, refused if any joint would change by
    // more than max_joint_change deg (default 5 + 0.5 deg per mm of travel).
    Joints plan_line(const Pose& pose, std::optional<double> max_joint_change = std::nullopt);
    // Straight-line TCP move (speed mm/s), sent in moveByLine's joint form. Returns the joint target.
    Joints move_line(const Pose& pose, double speed = 50.0, bool wait = true,
                     std::optional<double> max_joint_change = std::nullopt,
                     std::chrono::seconds timeout = std::chrono::seconds(60));
    void wait_until_stopped(std::chrono::seconds timeout, const std::optional<Joints>& q_target = std::nullopt,
                            double tol = 0.1);

    // --- timestamped trajectory (2.2.9) ----------------------------------------
    // times in s from 0, strictly increasing, >= 3 points. Moves to points[0] first.
    // `cancel` (optional) is checked between uploaded points and right before the start:
    // set it from another thread (e.g. a STOP button) and the upload is abandoned with an
    // ECError instead of the trajectory starting after the stop.
    void run_joint_trajectory(const std::vector<double>& times, const std::vector<Joints>& points,
                              double speed_percent = 100.0, double approach_speed = 10.0, bool wait = true,
                              std::chrono::seconds timeout = std::chrono::seconds(120),
                              const std::atomic<bool>* cancel = nullptr);
    void stop_trajectory();

    // --- kinematics (2.2.4), computed by the controller ------------------------
    Pose forward_kinematics(const Joints& q);
    Joints inverse_kinematics(const Pose& pose, const std::optional<Joints>& ref_q = std::nullopt);

    // --- streaming ----------------------------------------------------------
    std::unique_ptr<JointStreamer> stream(StreamerConfig config = {});

private:
    RpcClient rpc_;
    Monitor monitor_;
};

}  // namespace ec
