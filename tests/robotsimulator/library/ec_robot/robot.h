// High-level async interface to an Elite EC-series robot (EC SDK-Socket Manual Ver3.19.2).
//
// Units: joints deg, poses [x, y, z, rx, ry, rz] mm + rad (fixed-axis XYZ).
//
// Everything runs on ONE thread (the io_context's). Every operation returns a
// std::error_code (or a Result<T>); expected failures are never exceptions. The human-
// readable reason for the last error ("state is 'alarm'", "IK changes joint 4 by 31 deg")
// is in last_error_detail().
//
// Behaviour carried over from tests on controller V3.11.2:
//  - Cartesian targets are never sent as poses: moveByLine's `target_pose` form moved the
//    arm far from the target. move_line() converts the pose with the controller's IK,
//    checks the joint change, and sends `targetPos`.
//  - The brakes must be released on the teach pendant before power_on() can sync the
//    encoders ("the brake of robot is not opened").
//  - The active tool (TCP + payload) is selected per tool number; call set_tool().
#pragma once

#include <utility>   // before asio.hpp: Asio 1.18.1 uses std::exchange without including it
#include "asio.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "monitor.h"
#include "rpc_client.h"
#include "types.hpp"

namespace ec {

enum class RobotErrc {
    no_state = 1,          // no monitor packet, or the latest is older than 100 ms
    not_remote,            // switch to remote mode on the teach pendant
    servos_off,            // call power_on()
    not_synced,            // call power_on()
    not_stopped,           // the robot is moving, or in alarm / collision / e-stop
    alarm_not_cleared,
    sync_failed,           // usually: brakes not released on the pendant
    servos_did_not_start,
    rejected,              // the controller answered false
    tool_not_selected,
    ik_jump,               // IK solution changes a joint more than allowed for the move
    bad_trajectory,        // arguments: sizes, timestamps
    trajectory_rejected,   // check_trajectory failed
    motion_timeout,        // stop was sent
    motion_failed,         // ended in a state other than stop, or never started
    cancelled,
};
std::error_code make_error_code(RobotErrc e);

struct ToolInfo {
    Pose tcp{};                   // offset from the flange, mm / rad
    double mass = 0;              // kg
    std::array<double, 3> cog{};  // mm
};

class Robot {
public:
    Robot(asio::io_context& context, std::string ip = "192.168.0.11", bool strict_monitor = true);

    asio::awaitable<std::error_code> connect();   // 8055 + 8056
    void close();
    // The ports separately: mirroring needs only 8056, and the controller takes just one 8055 client
    asio::awaitable<std::error_code> connect_monitor();
    asio::awaitable<std::error_code> connect_rpc();
    void close_rpc() { rpc_.close(); }
    asio::io_context& context() { return context_; }
    RpcClient& rpc() { return rpc_; }
    Monitor& monitor() { return monitor_; }

    // --- state (synchronous: they only read the latest monitor packet) ------------------
    std::optional<RobotStatus> status() const { return monitor_.latest(); }
    std::string describe() const;
    std::error_code require_remote();
    std::error_code require_ready();   // fresh state, remote, servos on, synced, state stop
    const std::string& last_error_detail() const { return detail_; }

    asio::awaitable<Result<std::string>> version();

    // --- power / setup ---------------------------------------------------------------------
    asio::awaitable<std::error_code> power_on(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    asio::awaitable<std::error_code> power_off();
    asio::awaitable<std::error_code> set_speed(double percent);   // global override, 0.05..100 %
    asio::awaitable<std::error_code> set_tool(int tool_num);      // TCP + payload for remote mode
    asio::awaitable<Result<ToolInfo>> tool_info(int tool_num);
    asio::awaitable<std::error_code> stop();
    asio::awaitable<std::error_code> clear_alarm();
    asio::awaitable<std::error_code> stop_trajectory();

    // --- point-to-point ----------------------------------------------------------------------
    // Joint move, speed 0.01..100 %. With wait = true, completes when the arm has stopped.
    asio::awaitable<std::error_code> move_joint(Joints q, double speed = 10.0, bool wait = true,
                                                std::chrono::seconds timeout = std::chrono::seconds(60));
    // IK solution for a straight-line move to pose; refused (ik_jump) if a joint would change by
    // more than max_joint_change deg (default 5 + 0.5 deg per mm of travel).
    asio::awaitable<Result<Joints>> plan_line(Pose pose, std::optional<double> max_joint_change = std::nullopt);
    // Straight-line TCP move (speed mm/s), sent in moveByLine's joint form. Returns the joint target.
    asio::awaitable<Result<Joints>> move_line(Pose pose, double speed = 50.0, bool wait = true,
                                              std::optional<double> max_joint_change = std::nullopt,
                                              std::chrono::seconds timeout = std::chrono::seconds(60));
    asio::awaitable<std::error_code> wait_until_stopped(std::chrono::seconds timeout,
                                                        std::optional<Joints> q_target = std::nullopt,
                                                        double tol = 0.1);

    // --- timestamped trajectory --------------------------------------------------------------
    // times in s from 0, strictly increasing, >= 3 points. Moves to points[0] first.
    // Up to `upload_window` push_pos requests are in flight at once (1 = one at a time, as before).
    // `cancel` (optional) is checked between uploaded points and before the start: set it (on this
    // thread, e.g. from a STOP coroutine) and the upload ends with `cancelled` instead of starting.
    asio::awaitable<std::error_code> run_joint_trajectory(std::vector<double> times, std::vector<Joints> points,
                                                          double speed_percent = 100.0, double approach_speed = 10.0,
                                                          bool wait = true,
                                                          std::chrono::seconds timeout = std::chrono::seconds(120),
                                                          const bool* cancel = nullptr, int upload_window = 8);

    // --- kinematics, computed by the controller ----------------------------------------------
    asio::awaitable<Result<Pose>> forward_kinematics(Joints q);
    asio::awaitable<Result<Joints>> inverse_kinematics(Pose pose, std::optional<Joints> ref_q = std::nullopt);

private:
    std::error_code fail(RobotErrc e, std::string detail);
    std::error_code fail(std::error_code ec, std::string detail);
    // a call whose reply must not be `false`; its error (or `rejected`) goes to detail_
    asio::awaitable<std::error_code> command(std::string method, nlohmann::json params = nlohmann::json::array());

    struct Upload {   // push_pos requests in flight during run_joint_trajectory
        explicit Upload(asio::io_context& context) : wake(context) {}
        asio::steady_timer wake;   // cancelled whenever one request finishes
        int in_flight = 0;
        std::error_code first_error;
    };
    asio::awaitable<void> push_one(nlohmann::json params, Upload& upload);
    asio::awaitable<void> wait_for_slot(Upload& upload, int max_in_flight);

    asio::io_context& context_;
    RpcClient rpc_;
    Monitor monitor_;
    std::string detail_;
};

}  // namespace ec

template <>
struct std::is_error_code_enum<ec::RobotErrc> : std::true_type {};
