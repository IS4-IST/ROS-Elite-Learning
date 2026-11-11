#include "ros_interface.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <future>
#include <functional>
#include <thread>

using namespace std::chrono_literals;

namespace twin {
namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using sensor_msgs::msg::JointState;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

constexpr double kDeg = M_PI / 180.0;

// Start a core command and wait for its outcome (service callbacks run on executor threads,
// never on the core's own threads, so waiting here can't block the core)
Outcome wait_for_outcome(const std::function<void(Callback)>& start, std::chrono::seconds timeout, const char* what) {
    auto promise = std::make_shared<std::promise<Outcome>>();
    auto future = promise->get_future();
    start([promise](const Outcome& o) { promise->set_value(o); });
    if (future.wait_for(timeout) != std::future_status::ready)
        return Outcome{false, std::string(what) + ": no answer in " + std::to_string(timeout.count()) + " s"};
    return future.get();
}
const std::array<std::string, 6> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

double max_abs_diff(const ec::Joints& a, const ec::Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

std::vector<double> to_rad(const ec::Joints& q) {
    std::vector<double> v(6);
    for (int i = 0; i < 6; ++i) v[i] = q[i] * kDeg;
    return v;
}

KeyValue kv(std::string key, std::string value) {
    KeyValue k;
    k.key = std::move(key);
    k.value = std::move(value);
    return k;
}

}  // namespace

struct RosInterface::Impl {
    Core& core;
    rclcpp::Node::SharedPtr node;
    // The node only keeps WEAK references to its callback groups: they must live here,
    // or their timers/services/action silently stop (or crash the executor) once the
    // constructor returns.
    rclcpp::CallbackGroup::SharedPtr state_group, stop_group, service_group, action_group;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    std::thread spin_thread;

    rclcpp::Publisher<JointState>::SharedPtr joints_pub;
    rclcpp::Publisher<DiagnosticStatus>::SharedPtr status_pub;
    rclcpp::TimerBase::SharedPtr joints_timer, status_timer;
    rclcpp::Service<Trigger>::SharedPtr stop_srv, clear_srv;
    rclcpp::Service<SetBool>::SharedPtr power_srv;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action;

    std::mutex goal_mutex;
    std::shared_ptr<GoalHandle> active_goal;
    decltype(ec::RobotStatus::received) last_packet{};

    explicit Impl(Core& c) : core(c) {}

    void publish_joints();
    void publish_status();
    void accept_goal(const std::shared_ptr<GoalHandle>& goal);
    void finish(const std::shared_ptr<GoalHandle>& goal, const Outcome& outcome, bool started);
};

RosInterface::RosInterface(Core& core, int argc, char** argv) : impl_(std::make_unique<Impl>(core)) {
    // No ROS signal handler: the twin handles Ctrl+C itself (closes the window, stops a motion)
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    Impl& d = *impl_;
    d.node = std::make_shared<rclcpp::Node>("digitaltwin");

    // Callback groups: a slow service (power on takes seconds) must never hold up /stop,
    // the joint states or the action.
    d.state_group = d.node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    d.stop_group = d.node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    d.service_group = d.node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    d.action_group = d.node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // --- publishers ---------------------------------------------------------------------
    d.joints_pub = d.node->create_publisher<JointState>("joint_states", 10);
    d.status_pub = d.node->create_publisher<DiagnosticStatus>("robot_status", 10);
    d.joints_timer = d.node->create_wall_timer(2ms, [&d] { d.publish_joints(); }, d.state_group);
    d.status_timer = d.node->create_wall_timer(100ms, [&d] { d.publish_status(); }, d.state_group);

    // --- services -----------------------------------------------------------------------
    d.stop_srv = d.node->create_service<Trigger>(
        "stop",
        [&d](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
            const Outcome o = wait_for_outcome([&d](Callback cb) { d.core.stop("ROS /stop", std::move(cb)); }, 5s, "stop");
            res->success = o.ok;
            res->message = o.message;
        },
        rmw_qos_profile_services_default, d.stop_group);
    d.power_srv = d.node->create_service<SetBool>(
        "power",
        [&d](const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> res) {
            const bool on = req->data;
            const Outcome o = wait_for_outcome([&d, on](Callback cb) { d.core.power(on, Source::External, std::move(cb)); },
                                               60s, "power");
            res->success = o.ok;
            res->message = o.message;
        },
        rmw_qos_profile_services_default, d.service_group);
    d.clear_srv = d.node->create_service<Trigger>(
        "clear_alarm",
        [&d](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
            const Outcome o = wait_for_outcome([&d](Callback cb) { d.core.clear_alarm(Source::External, std::move(cb)); },
                                               10s, "clear alarm");
            res->success = o.ok;
            res->message = o.message;
        },
        rmw_qos_profile_services_default, d.service_group);

    // --- action ---------------------------------------------------------------------------
    d.action = rclcpp_action::create_server<FollowJointTrajectory>(
        d.node, "follow_joint_trajectory",
        // every goal is accepted, then checked, so a refusal can carry its reason (error_string)
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&d](const std::shared_ptr<GoalHandle> goal) {
            {
                std::lock_guard<std::mutex> lock(d.goal_mutex);
                if (goal != d.active_goal) return rclcpp_action::CancelResponse::REJECT;
            }
            d.core.cancel_motion("the ROS client cancelled the goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&d](const std::shared_ptr<GoalHandle> goal) { d.accept_goal(goal); },
        rcl_action_server_get_default_options(), d.action_group);

    d.executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
    d.executor->add_node(d.node);
    d.spin_thread = std::thread([&d] { d.executor->spin(); });
    RCLCPP_INFO(d.node->get_logger(),
                "digitaltwin: /joint_states, /robot_status, /stop, /power, /clear_alarm, /follow_joint_trajectory");
}

RosInterface::~RosInterface() {
    Impl& d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.goal_mutex);
        if (d.active_goal) d.core.cancel_motion("the twin is closing");
    }
    d.executor->cancel();
    if (d.spin_thread.joinable()) d.spin_thread.join();
    rclcpp::shutdown();
}

// --- publishers ------------------------------------------------------------------------------
void RosInterface::Impl::publish_joints() {
    const auto s = core.status();
    if (!s || s->received == last_packet) return;   // one message per monitor packet
    last_packet = s->received;
    JointState msg;
    msg.header.stamp = node->now();
    msg.name.assign(kJointNames.begin(), kJointNames.end());
    msg.position.resize(6);
    msg.velocity.resize(6);
    msg.effort.resize(6);
    for (int i = 0; i < 6; ++i) {
        msg.position[i] = s->joint_pos[i] * kDeg;
        msg.velocity[i] = s->joint_speed[i] * kDeg;
        msg.effort[i] = s->joint_torque[i];
    }
    joints_pub->publish(msg);
}

void RosInterface::Impl::publish_status() {
    const auto s = core.status();
    const bool fresh = Core::fresh(s);
    DiagnosticStatus msg;
    msg.name = "ec66";
    msg.hardware_id = core.ip();
    if (!fresh) {
        msg.level = DiagnosticStatus::ERROR;
        msg.message = s ? "robot state is stale" : "no robot state";
    } else {
        const auto st = s->robot_state;
        const bool fault = st == ec::State::Alarm || st == ec::State::Collision || st == ec::State::EStop || s->estop;
        msg.level = fault ? DiagnosticStatus::ERROR
                  : (!s->servo_ready || s->robot_mode != ec::Mode::Remote) ? DiagnosticStatus::WARN
                  : DiagnosticStatus::OK;
        msg.message = ec::to_string(st);
        msg.values.push_back(kv("mode", ec::to_string(s->robot_mode)));
        msg.values.push_back(kv("state", ec::to_string(st)));
        msg.values.push_back(kv("servo", s->servo_ready ? "on" : "off"));
        msg.values.push_back(kv("synced", s->motor_synced ? "yes" : "no"));
        msg.values.push_back(kv("estop", s->estop ? "pressed" : "no"));
        msg.values.push_back(kv("collision", s->collision ? "yes" : "no"));
    }
    const View v = core.view();
    msg.values.push_back(kv("connected_8055", v.rpc_connected ? "yes" : "no"));
    msg.values.push_back(kv("armed", v.armed ? "yes" : "no"));
    msg.values.push_back(kv("external_control", v.external ? "granted" : "not granted"));
    msg.values.push_back(kv("active_motion", to_string(v.active)));
    msg.values.push_back(kv("progress", std::to_string(v.progress)));
    status_pub->publish(msg);
}

// --- action ------------------------------------------------------------------------------------
void RosInterface::Impl::finish(const std::shared_ptr<GoalHandle>& goal, const Outcome& outcome, bool started) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_string = outcome.message;
    if (outcome.ok) {
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal->succeed(result);
    } else {
        result->error_code = started ? FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED
                                     : FollowJointTrajectory::Result::INVALID_GOAL;
        if (goal->is_canceling()) goal->canceled(result);
        else goal->abort(result);
    }
    RCLCPP_INFO(node->get_logger(), "goal %s: %s", outcome.ok ? "succeeded" : "ended", outcome.message.c_str());
    std::lock_guard<std::mutex> lock(goal_mutex);
    if (active_goal == goal) active_goal.reset();
}

void RosInterface::Impl::accept_goal(const std::shared_ptr<GoalHandle>& goal) {
    const auto& traj = goal->get_goal()->trajectory;
    auto refuse = [&](const std::string& why) { finish(goal, {false, "refused: " + why}, false); };

    // joint order
    std::array<int, 6> index{0, 1, 2, 3, 4, 5};   // message column -> our joint
    if (!traj.joint_names.empty()) {
        if (traj.joint_names.size() != 6) return refuse("need the 6 joints joint1..joint6");
        for (int c = 0; c < 6; ++c) {
            const auto it = std::find(kJointNames.begin(), kJointNames.end(), traj.joint_names[c]);
            if (it == kJointNames.end()) return refuse("unknown joint '" + traj.joint_names[c] + "'");
            index[c] = static_cast<int>(it - kJointNames.begin());
        }
    }
    if (traj.points.empty()) return refuse("the trajectory has no points");

    const auto s = core.status();
    if (!Core::fresh(s)) return refuse("no fresh robot state");
    const ec::Joints current = s->q();

    std::vector<ec::Joints> points;
    std::vector<double> times;
    bool timed = false;
    for (const auto& p : traj.points) {
        if (p.positions.size() != 6) return refuse("every point needs 6 positions");
        ec::Joints q;
        for (int c = 0; c < 6; ++c) q[index[c]] = p.positions[c] / kDeg;
        points.push_back(q);
        const double t = rclcpp::Duration(p.time_from_start).seconds();
        times.push_back(t);
        if (t > 0) timed = true;
    }

    // Start from where the arm is. (The twin collision-checks this first segment too.)
    if (timed) {
        if (times.front() > 0) {
            points.insert(points.begin(), current);
            times.insert(times.begin(), 0.0);
        } else if (max_abs_diff(points.front(), current) > 0.5) {
            return refuse("the first point (t = 0) is not where the arm is: plan again");
        }
    } else if (max_abs_diff(points.front(), current) > 0.01) {
        points.insert(points.begin(), current);
    }

    bool busy;
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        busy = active_goal != nullptr;
        if (!busy) active_goal = goal;
    }
    if (busy) return refuse("another goal is running");   // outside the lock: finish() takes it

    MotionCallbacks cb;
    cb.feedback = [this, goal](double t, const ec::Joints& desired, const ec::Joints& actual) {
        auto fb = std::make_shared<FollowJointTrajectory::Feedback>();
        fb->header.stamp = node->now();
        fb->joint_names.assign(kJointNames.begin(), kJointNames.end());
        fb->desired.positions = to_rad(desired);
        fb->actual.positions = to_rad(actual);
        fb->error.positions.resize(6);
        for (int i = 0; i < 6; ++i) fb->error.positions[i] = (desired[i] - actual[i]) * kDeg;
        fb->desired.time_from_start = rclcpp::Duration::from_seconds(t);
        goal->publish_feedback(fb);
    };
    // refusals and results both arrive here; Outcome::started tells them apart
    cb.done = [this, goal](const Outcome& o) { finish(goal, o, o.started); };

    if (timed) {
        ec::planning::Trajectory t;
        t.times = std::move(times);
        t.points = std::move(points);
        core.run_timed(std::move(t), Source::External, std::move(cb));
    } else {
        core.run_path(std::move(points), Source::External, std::move(cb));
    }
}

}  // namespace twin
