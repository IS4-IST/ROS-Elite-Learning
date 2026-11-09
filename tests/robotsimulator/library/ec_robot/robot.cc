#include "ec_robot/robot.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ec {
namespace {

Joints to_joints(const nlohmann::json& j, const char* what) {
    if (!j.is_array() || j.size() < 6) throw ECError(std::string(what) + " returned " + j.dump());
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = j[i].get<double>();
    return q;
}

double max_abs_diff(const Joints& a, const Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

}  // namespace

Robot::Robot(std::string ip, bool strict_monitor) : rpc_(ip), monitor_(ip, Monitor::kPort, strict_monitor) {}

Robot::~Robot() { close(); }

void Robot::connect() {
    rpc_.connect();
    monitor_.start();
}

void Robot::close() {
    monitor_.stop();
    rpc_.close();
}

RobotStatus Robot::status(std::chrono::milliseconds max_age) const { return monitor_.latest(max_age); }

std::string Robot::version() {
    auto v = rpc_.call("getSoftVersion");
    return v.is_string() ? v.get<std::string>() : v.dump();
}

std::string Robot::describe() const {
    const auto s = status();
    std::ostringstream o;
    o << "mode=" << to_string(s.robot_mode) << " state=" << to_string(s.robot_state)
      << " servo=" << (s.servo_ready ? "on" : "off") << " synced=" << (s.motor_synced ? "yes" : "no");
    return o.str();
}

void Robot::require_remote() const {
    if (status().robot_mode != Mode::Remote)
        throw NotReady("robot is not in REMOTE mode (switch on the teach pendant)");
}

void Robot::require_ready() const {
    require_remote();
    const auto s = status();
    if (!s.servo_ready) throw NotReady("servos are off (call power_on())");
    if (!s.motor_synced) throw NotReady("encoders not synced (call power_on())");
    if (s.robot_state != State::Stop)
        throw NotReady(std::string("robot state is '") + to_string(s.robot_state) + "', needs 'stop'");
}

void Robot::power_on(std::chrono::milliseconds timeout) {
    require_remote();
    if (status().robot_state != State::Stop) {
        rpc_.call("clearAlarm");
        if (!monitor_.wait_for([](const RobotStatus& s) { return s.robot_state == State::Stop; }, timeout))
            throw NotReady("alarm not cleared (" + describe() + "); check the pendant");
    }
    if (!status().motor_synced) {
        rpc_.call("syncMotorStatus");   // fails until the brakes are released on the pendant
        if (!monitor_.wait_for([](const RobotStatus& s) { return s.motor_synced; }, timeout))
            throw NotReady("encoder sync failed");
    }
    if (!status().servo_ready) {
        rpc_.call("set_servo_status", {{"status", 1}});
        if (!monitor_.wait_for([](const RobotStatus& s) { return s.servo_ready; }, timeout))
            throw NotReady("servos did not switch on");
    }
}

void Robot::power_off() {
    rpc_.call("set_servo_status", {{"status", 0}});
    monitor_.wait_for([](const RobotStatus& s) { return !s.servo_ready; }, std::chrono::milliseconds(5000));
}

void Robot::set_speed(double percent) {
    require_remote();
    rpc_.call("setSpeed", {{"value", percent}});
}

void Robot::set_tool(int tool_num) {
    require_remote();
    rpc_.call("setAutoRunToolNumber", {{"tool_num", tool_num}});
    if (rpc_.call("getAutoRunToolNumber") != tool_num)
        throw ECError("tool " + std::to_string(tool_num) + " was not selected");
}

ToolInfo Robot::tool_info(int tool_num) {
    ToolInfo t;
    auto tcp = rpc_.call("getTcpPos", {{"tool_num", tool_num}, {"unit_type", 1}});
    for (int i = 0; i < 6 && i < static_cast<int>(tcp.size()); ++i) t.tcp[i] = tcp[i].get<double>();
    t.mass = rpc_.call("getPayload", {{"tool_num", tool_num}}).get<double>();
    auto cog = rpc_.call("getCentreMass", {{"tool_num", tool_num}});
    for (int i = 0; i < 3 && i < static_cast<int>(cog.size()); ++i) t.cog[i] = cog[i].get<double>();
    return t;
}

void Robot::stop() { rpc_.call("stop"); }

void Robot::wait_until_stopped(std::chrono::seconds timeout, const std::optional<Joints>& q_target, double tol) {
    auto started = monitor_.wait_for([](const RobotStatus& s) { return s.robot_state != State::Stop; },
                                     std::chrono::milliseconds(500));
    if (!started && q_target && max_abs_diff(status().q(), *q_target) <= tol) return;   // already there
    auto s = monitor_.wait_for([](const RobotStatus& st) { return st.robot_state != State::Running; },
                               std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
    if (!s) {
        stop();
        throw ECError("motion did not finish in time; stop sent");
    }
    if (s->robot_state != State::Stop)
        throw ECError(std::string("motion ended in state '") + to_string(s->robot_state) + "'");
}

void Robot::move_joint(const Joints& q, double speed, bool wait, std::chrono::seconds timeout) {
    require_ready();
    if (rpc_.call("moveByJoint", {{"targetPos", q}, {"speed", speed}}) == false)
        throw ECError("moveByJoint rejected by the controller");
    if (wait) wait_until_stopped(timeout, q);
}

Joints Robot::plan_line(const Pose& pose, std::optional<double> max_joint_change) {
    const auto s = status();
    const Joints q_now = s.q();
    const Joints q_target = inverse_kinematics(pose, q_now);
    const double dist = std::sqrt(std::pow(pose[0] - s.tcp_pose[0], 2) + std::pow(pose[1] - s.tcp_pose[1], 2) +
                                  std::pow(pose[2] - s.tcp_pose[2], 2));
    const double limit = max_joint_change.value_or(5.0 + 0.5 * dist);
    const double change = max_abs_diff(q_now, q_target);
    if (change > limit) {
        throw ECError("IK solution changes a joint by " + std::to_string(change) + " deg for a " +
                      std::to_string(dist) + " mm move (limit " + std::to_string(limit) + "); refusing");
    }
    return q_target;
}

Joints Robot::move_line(const Pose& pose, double speed, bool wait, std::optional<double> max_joint_change,
                        std::chrono::seconds timeout) {
    const Joints q_target = plan_line(pose, max_joint_change);
    require_ready();
    // joint form only: the target_pose form misbehaved on V3.11.2
    if (rpc_.call("moveByLine", {{"targetPos", q_target}, {"speed", speed}}) == false)
        throw ECError("moveByLine rejected by the controller");
    if (wait) wait_until_stopped(timeout, q_target);
    return q_target;
}

void Robot::run_joint_trajectory(const std::vector<double>& times, const std::vector<Joints>& points,
                                 double speed_percent, double approach_speed, bool wait,
                                 std::chrono::seconds timeout) {
    if (times.size() != points.size() || points.size() < 3)
        throw std::invalid_argument("need >= 3 points and one timestamp per point");
    if (times.front() != 0.0) throw std::invalid_argument("timestamps must start at 0");
    for (std::size_t i = 1; i < times.size(); ++i) {
        if (times[i] <= times[i - 1]) throw std::invalid_argument("timestamps must increase");
    }
    move_joint(points.front(), approach_speed);
    rpc_.call("flush_trajectory");
    rpc_.call("start_push_pos", {{"path_lenth", points.size()},   // 'lenth' as in the manual
                                 {"pos_type", 0},
                                 {"ref_joint_pos", points.front()},
                                 {"ref_frame", Pose{}},
                                 {"ret_flag", 1}});
    for (std::size_t i = 0; i < points.size(); ++i) {
        rpc_.call("push_pos", {{"timestamp", times[i]}, {"pos", points[i]}});
    }
    if (rpc_.call("stop_push_pos") == false) throw ECError("stop_push_pos: not all points were accepted");
    const int check = rpc_.call("check_trajectory").get<int>();
    if (check != 0) {
        const char* reason = check == -1 ? "point count mismatch" : check == -2 ? "point format error"
                           : check == -3 ? "timestamps invalid" : "unknown";
        throw ECError(std::string("trajectory rejected: ") + reason);
    }
    if (rpc_.call("start_trajectory", {{"speed_percent", speed_percent}}) == false)
        throw ECError("start_trajectory rejected");
    if (wait) wait_until_stopped(timeout, points.back());
}

void Robot::stop_trajectory() { rpc_.call("stop_trajectory"); }

Pose Robot::forward_kinematics(const Joints& q) {
    return to_joints(rpc_.call("positiveKinematic", {{"targetPos", q}, {"unit_type", 1}}), "positiveKinematic");
}

Joints Robot::inverse_kinematics(const Pose& pose, const std::optional<Joints>& ref_q) {
    nlohmann::json params = {{"targetPose", pose}, {"unit_type", 1}};
    if (ref_q) params["referencePos"] = *ref_q;
    return to_joints(rpc_.call("inverseKinematic", params), "inverseKinematic");
}

std::unique_ptr<JointStreamer> Robot::stream(StreamerConfig config) {
    return std::make_unique<JointStreamer>(*this, config);
}

}  // namespace ec
