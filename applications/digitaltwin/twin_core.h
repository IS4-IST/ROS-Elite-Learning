// The digital twin's core: the ONE place that commands the robot.
//
// The GUI and the ROS interface are both front ends of this object. It owns the robot
// connection, the operator's permissions and the motion that is running, and every
// command, whoever sends it, goes through the same checks:
//
//   operator (GUI)       needs "arm commands"; locked out while an external motion runs
//   external (ROS)       needs "arm commands" AND "accept external commands"
//   stop                 anyone, always (as long as 8055 is connected)
//
// The permissions drop by themselves (checked every 50 ms on the robot thread): arming needs
// 8055, a fresh monitor (< 200 ms) and remote mode; the external grant needs arming.
// Withdrawing the grant while an external motion runs stops it.
//
// Threads. The core runs two of its own; callers (GUI, ROS) never block on either:
//   robot thread     an io_context with the async ec::Robot (8055 + 8056) and every command
//                    and motion as a coroutine. Nothing slow runs here: STOP, the monitor
//                    and a running motion share it.
//   planning thread  OMPL planning and the collision checks of every motion (verify):
//                    tens of ms to seconds of CPU work, kept off the robot thread.
// Commands are posted to the robot thread and return at once; their outcome arrives through
// an optional callback (called on the robot thread) and in view().last. view() is a
// snapshot for any thread.
//
// Motions are joint trajectories. Before anything is sent the planning thread times them
// (geometric paths) or checks their timing (timed ones) against the motion limits,
// densifies them to 8 ms and checks every sample and the motion between samples for
// collisions in the twin's own scene. Back on the robot thread: arm still at the start
// (0.5 deg)? upload (STOP overtakes it and cancels it) -> run -> feedback ~20 Hz -> result.
#pragma once

#include <mujoco/mujoco.h>

#include <utility>   // before asio.hpp: Asio 1.18.1 uses std::exchange without including it
#include "asio.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ec_robot/robot.h"
#include "planning/planner.h"

namespace twin {

using Clock = std::chrono::steady_clock;
using ec::Joints;

struct Outcome {
    bool ok = false;
    std::string message;
    bool started = false;   // for motions: true once the trajectory was handed to the controller
};
using Callback = std::function<void(const Outcome&)>;   // called on the robot thread

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
    Callback done;   // exactly once: refused, finished, failed or stopped
};

// Everything a front end shows, copied in one go (any thread)
struct View {
    std::optional<ec::RobotStatus> status;   // latest monitor packet, possibly stale
    bool monitor_connected = false;
    bool connecting = false;                 // a connect is in progress
    std::uint64_t packets = 0, invalid_packets = 0;
    bool rpc_connected = false;
    bool armed = false, external = false, can_arm = false;
    Source active = Source::None;
    double progress = 0;                     // 0..1 of the running motion
    std::string motion_text;
    bool busy = false;                       // an operator command or a plan is in progress
    Outcome last;                            // the last finished command, for display
    std::shared_ptr<const Plan> plan;        // the operator's plan, if any
};

class Core {
public:
    Core(const mjModel* model, std::string ip, ec::planning::Options plan_options);
    ~Core();
    Core(const Core&) = delete;
    Core& operator=(const Core&) = delete;

    const std::string& ip() const { return ip_; }
    View view() const;
    std::optional<ec::RobotStatus> status() const { return robot_->status(); }   // any thread
    static bool fresh(const std::optional<ec::RobotStatus>& s) { return s && s->age_seconds() < 0.2; }

    // --- connections ----------------------------------------------------------------------
    void connect_monitor();
    void connect_robot();
    void disconnect_robot();

    // --- permissions (the operator's) --------------------------------------------------------
    void set_armed(bool on);
    void set_external_allowed(bool on);

    // --- commands: return at once; the outcome goes to `done` and view().last ---------------
    void stop(const std::string& who, Callback done = {});     // anyone, always
    void power(bool on, Source from, Callback done = {});      // on: tool + speed override from settings
    void clear_alarm(Source from, Callback done = {});
    void set_speed(double percent, Callback done = {});        // operator
    void move_joint(const Joints& q, double speed, Callback done = {});   // operator, one moveByJoint

    // Motions. Refusals also come back through cb.done (with started = false).
    void run_plan(std::shared_ptr<const Plan> plan, Source from, MotionCallbacks cb);
    void run_path(std::vector<Joints> waypoints, Source from, MotionCallbacks cb);   // the twin times it
    void run_timed(ec::planning::Trajectory traj, Source from, MotionCallbacks cb);  // checked against the limits
    void cancel_motion(const std::string& reason);

    // --- the operator's planning (OMPL) ---------------------------------------------------------
    void plan_to(const Joints& goal);   // result in view().plan / view().last
    void discard_plan();

    void set_limits(double speed, double acc);              // motion limits, every joint
    void set_power_settings(int tool, double speed_override);

private:
    // all of these run on the robot thread
    bool allowed(Source from, std::string* why) const;
    bool can_arm() const;
    void finish(const Outcome& o, const Callback& done);
    void command(std::string what, Source from, bool needs_permission,
                 std::function<asio::awaitable<std::error_code>()> body, Callback done);
    void start_motion(std::function<std::shared_ptr<const Plan>()> build, Source from, MotionCallbacks cb);
    asio::awaitable<void> motion(std::function<std::shared_ptr<const Plan>()> build, Source from, MotionCallbacks cb);
    asio::awaitable<Outcome> execute(const Plan& plan, Source from, const MotionCallbacks& cb);
    asio::awaitable<void> watch_permissions();
    void stop_with(const std::string& label, const std::string& reason, Callback done);

    // planning thread: runs `fn` there and resumes the calling coroutine on the robot thread
    template <typename T>
    asio::awaitable<T> on_planning_thread(std::function<T()> fn);
    std::shared_ptr<Plan> describe(std::string source, ec::planning::Trajectory traj) const;   // planning thread

    std::string ip_;
    ec::planning::Planner planner_;   // planning thread only

    asio::io_context robot_context_, planning_context_;
    asio::executor_work_guard<asio::io_context::executor_type> robot_work_, planning_work_;
    std::unique_ptr<ec::Robot> robot_;   // robot thread (status() is safe from any thread)

    // shared with readers on other threads
    mutable std::mutex mutex_;
    Outcome last_;
    std::string motion_text_;
    std::shared_ptr<const Plan> plan_;
    ec::planning::Limits limits_;
    int power_tool_ = 1;
    double power_speed_ = 20;   // %
    std::atomic<bool> armed_{false}, external_{false}, connecting_{false};
    std::atomic<Source> active_{Source::None};
    std::atomic<double> progress_{0};
    std::atomic<int> busy_{0};

    // robot thread only
    bool cancel_ = false;
    std::string cancel_reason_;

    std::thread robot_thread_, planning_thread_;   // last: start after everything above exists
};

}  // namespace twin
