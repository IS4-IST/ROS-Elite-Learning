// The digital twin's core: the ONE place that commands the robot.
//
// The GUI and the ROS interface are both front ends of this object. It owns the 8055
// connection, the operator's permissions and the motion that is running, and every
// command, whoever sends it, goes through the same checks:
//
//   operator (GUI)       needs "arm commands"; locked out while an external motion runs
//   external (ROS)       needs "arm commands" AND "accept external commands"
//   stop                 anyone, always (as long as 8055 is connected)
//
// The permissions drop by themselves (update_permissions, called every frame): arming
// needs 8055, a fresh monitor (< 200 ms) and remote mode; the external grant needs arming.
// Withdrawing the grant while an external motion runs stops it.
//
// Motions are joint trajectories. Before anything is sent the core times them (geometric
// paths) or checks their timing (timed ones) against the motion limits, densifies them to
// 8 ms and checks every sample and the motion between samples for collisions in the
// twin's own scene. Then: arm still at the start (0.5 deg)? upload (STOP cancels the
// upload) -> run -> feedback ~20 Hz -> result.
#pragma once

#include <mujoco/mujoco.h>
#include "asio.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ec_robot/monitor.hpp"
#include "ec_robot/robot.hpp"
#include "planning/planner.h"

namespace twin {

using Clock = std::chrono::steady_clock;
using ec::Joints;

struct Outcome {
    bool ok = false;
    std::string message;
};

// Runs blocking work off the caller's thread, one job at a time.
class Worker {
public:
    Worker();
    ~Worker();
    bool busy() const { return busy_; }
    // false (and nothing runs) if a job is already running. `done` runs on the worker.
    bool run(std::string what, std::function<void()> fn, std::function<void(const Outcome&)> done = {});
    Outcome last() const;   // result of the last finished job

private:
    asio::io_context io_;
    asio::executor_work_guard<asio::io_context::executor_type> guard_{io_.get_executor()};
    std::atomic<bool> busy_{false};
    mutable std::mutex mutex_;
    Outcome last_;
    std::thread thread_;   // last: starts after everything above exists
};

enum class Source { None, Operator, External };
const char* to_string(Source s);

// A motion ready to run: densified, timed, collision-checked
struct Plan {
    std::string source;   // where it came from, for display
    ec::planning::Trajectory traj;
    std::string summary;
    Clock::time_point created = Clock::now();
};

struct MotionCallbacks {
    // t since start, desired and measured joints (deg); ~20 Hz while running
    std::function<void(double t, const Joints& desired, const Joints& actual)> feedback;
    std::function<void(const Outcome&)> done;   // exactly once, if the motion was accepted
};

class Core {
public:
    Core(const mjModel* model, std::string ip, ec::planning::Options plan_options);
    ~Core();

    const std::string& ip() const { return ip_; }

    // --- observation (8056) -------------------------------------------------------------
    void connect_monitor();                        // (re)connect, on the jobs worker
    std::shared_ptr<ec::Monitor> monitor();
    std::optional<ec::RobotStatus> status();       // latest packet, possibly stale
    static bool fresh(const std::optional<ec::RobotStatus>& s) { return s && s->age_seconds() < 0.2; }

    // --- 8055 ---------------------------------------------------------------------------
    std::shared_ptr<ec::Robot> robot();
    void connect_robot();
    void disconnect_robot();

    // --- permissions (set by the operator) ----------------------------------------------
    bool can_arm();
    bool armed() const { return armed_; }
    void set_armed(bool on);
    bool external_allowed() const { return external_; }
    void set_external_allowed(bool on);
    void update_permissions();                     // drops what no longer holds

    // --- motion state ---------------------------------------------------------------------
    Source active() const { return active_; }
    double progress() const { return progress_; }  // 0..1 of the running motion
    std::string motion_text() const;

    // --- commands -------------------------------------------------------------------------
    // Immediate refusals come back in the future right away; otherwise when done.
    std::future<Outcome> stop(const std::string& who);   // anyone; sets the cancel flag at once
    std::future<Outcome> power(bool on, Source from);    // on: tool + speed override from power_settings
    void set_power_settings(int tool, double speed_override);
    std::future<Outcome> clear_alarm(Source from);
    std::future<Outcome> set_speed(double percent);                    // operator
    std::future<Outcome> move_joint(const Joints& q, double speed);    // operator, single moveByJoint

    // Motions. false + why if refused at once; otherwise callbacks.done fires later.
    bool run_plan(std::shared_ptr<const Plan> plan, Source from, MotionCallbacks cb, std::string* why);
    bool run_path(std::vector<Joints> waypoints, Source from, MotionCallbacks cb, std::string* why);
    bool run_timed(ec::planning::Trajectory traj, Source from, MotionCallbacks cb, std::string* why);
    void cancel_motion(const std::string& reason); // stop + cancel the upload

    // --- operator planning (OMPL) -----------------------------------------------------------
    std::future<Outcome> plan_to(const Joints& goal);   // result in plan()
    std::shared_ptr<const Plan> plan();
    void discard_plan();

    // Motion limits: used to time geometric paths and as the maximum for timed ones
    ec::planning::Limits limits();
    void set_limits(double speed, double acc);

    bool busy() const { return jobs_.busy(); }
    Outcome last_result() const { return jobs_.last(); }
    const ec::planning::Planner& planner() const { return planner_; }

private:
    bool allowed(Source from, std::string* why);
    std::future<Outcome> stop_with(const std::string& label, const std::string& reason);
    std::future<Outcome> submit(const std::string& what, Source from, bool needs_permission,
                                std::function<void()> fn);
    bool start_motion(std::function<std::shared_ptr<const Plan>()> build, Source from, MotionCallbacks cb,
                      std::string* why);
    void execute(const Plan& plan, Source from, const MotionCallbacks& cb);   // on the jobs worker
    std::shared_ptr<Plan> describe(std::string source, ec::planning::Trajectory traj) const;

    std::string ip_;
    ec::planning::Planner planner_;   // only used on the jobs worker

    mutable std::mutex mutex_;
    std::shared_ptr<ec::Monitor> monitor_;
    std::shared_ptr<ec::Robot> robot_;
    std::shared_ptr<const Plan> plan_;
    ec::planning::Limits limits_;
    std::string cancel_reason_;
    std::string motion_text_;
    int power_tool_ = 1;
    double power_speed_ = 20;   // %

    std::atomic<bool> armed_{false}, external_{false};
    std::atomic<Source> active_{Source::None};
    std::atomic<double> progress_{0};
    std::atomic<bool> cancel_{false};

    Worker jobs_;     // connect, power, plans, motions (one at a time)
    Worker urgent_;   // stop only
};

}  // namespace twin
