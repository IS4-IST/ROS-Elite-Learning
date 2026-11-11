#include "robot.h"

#include <cmath>
#include <sstream>

// GCC 11-13 coroutine pitfalls, avoided throughout this file:
//  - no braced lists inside a co_await expression: build the json argument first
//  - no structured bindings on a co_await result: name it and use .ec / .value / .first

namespace ec {
namespace {

class RobotCategory : public std::error_category {
public:
    const char* name() const noexcept override { return "ec-robot"; }
    std::string message(int code) const override {
        switch (static_cast<RobotErrc>(code)) {
            case RobotErrc::no_state: return "no fresh robot state (monitor)";
            case RobotErrc::not_remote: return "robot is not in REMOTE mode (switch on the teach pendant)";
            case RobotErrc::servos_off: return "servos are off (power on first)";
            case RobotErrc::not_synced: return "encoders not synced (power on first)";
            case RobotErrc::not_stopped: return "robot state is not 'stop'";
            case RobotErrc::alarm_not_cleared: return "alarm not cleared; check the pendant";
            case RobotErrc::sync_failed: return "encoder sync failed (brakes released on the pendant?)";
            case RobotErrc::servos_did_not_start: return "servos did not switch on";
            case RobotErrc::rejected: return "rejected by the controller";
            case RobotErrc::tool_not_selected: return "tool was not selected";
            case RobotErrc::ik_jump: return "IK solution changes a joint too much for this move";
            case RobotErrc::bad_trajectory: return "invalid trajectory arguments";
            case RobotErrc::trajectory_rejected: return "trajectory rejected by the controller";
            case RobotErrc::motion_timeout: return "motion did not finish in time; stop sent";
            case RobotErrc::motion_failed: return "motion did not complete";
            case RobotErrc::cancelled: return "cancelled";
        }
        return "unknown robot error";
    }
};

double max_abs_diff(const Joints& a, const Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

// a json array of 6 numbers -> Joints/Pose; false if it isn't one
bool to_six(const nlohmann::json& v, std::array<double, 6>& out) {
    if (!v.is_array() || v.size() < 6) return false;
    for (int i = 0; i < 6; ++i) {
        if (!v[i].is_number()) return false;
        out[i] = v[i].get<double>();
    }
    return true;
}

}  // namespace

std::error_code make_error_code(RobotErrc e) {
    static RobotCategory category;
    return {static_cast<int>(e), category};
}

Robot::Robot(asio::io_context& context, std::string ip, bool strict_monitor)
    : context_(context), rpc_(context, ip), monitor_(context, ip, Monitor::kPort, strict_monitor) {}

std::error_code Robot::fail(RobotErrc e, std::string detail) { return fail(make_error_code(e), std::move(detail)); }

std::error_code Robot::fail(std::error_code ec, std::string detail) {
    detail_ = detail.empty() ? ec.message() : std::move(detail);
    return ec;
}

asio::awaitable<std::error_code> Robot::command(std::string method, nlohmann::json params) {
    const Reply r = co_await rpc_.call(method, params);
    if (r.ec) co_return fail(r.ec, method + ": " + (r.value.is_object() ? r.value.value("message", r.ec.message())
                                                                          : r.ec.message()));
    if (r.value == false) co_return fail(RobotErrc::rejected, method + " rejected by the controller");
    co_return std::error_code{};
}

// --- connection / state -----------------------------------------------------------------------
asio::awaitable<std::error_code> Robot::connect() {
    std::error_code ec = co_await rpc_.connect();
    if (ec) co_return fail(ec, "8055: " + ec.message());
    ec = co_await monitor_.connect();
    if (ec) {
        rpc_.close();
        co_return fail(ec, "8056: " + ec.message());
    }
    co_return std::error_code{};
}

asio::awaitable<std::error_code> Robot::connect_monitor() {
    const std::error_code ec = co_await monitor_.connect();
    co_return ec ? fail(ec, "8056: " + ec.message()) : std::error_code{};
}

asio::awaitable<std::error_code> Robot::connect_rpc() {
    std::error_code ec = co_await rpc_.connect();
    if (ec) co_return fail(ec, "8055: " + ec.message());
    // A controller that already has a client accepts the connection and closes it at once:
    // only an answer proves this connection is ours.
    const Reply r = co_await rpc_.call("getSoftVersion");
    if (r.ec) {
        rpc_.close();
        co_return fail(r.ec, "8055 closed the connection: another client holds it");
    }
    co_return std::error_code{};
}

void Robot::close() {
    monitor_.close();
    rpc_.close();
}

std::string Robot::describe() const {
    const auto s = status();
    if (!s) return "no state";
    std::ostringstream o;
    o << "mode=" << to_string(s->robot_mode) << " state=" << to_string(s->robot_state)
      << " servo=" << (s->servo_ready ? "on" : "off") << " synced=" << (s->motor_synced ? "yes" : "no");
    return o.str();
}

std::error_code Robot::require_remote() {
    const auto s = status();
    if (!s || s->age_seconds() > 0.1) return fail(RobotErrc::no_state, "");
    if (s->robot_mode != Mode::Remote) return fail(RobotErrc::not_remote, "");
    return {};
}

std::error_code Robot::require_ready() {
    if (std::error_code ec = require_remote()) return ec;
    const auto s = status();
    if (!s->servo_ready) return fail(RobotErrc::servos_off, "");
    if (!s->motor_synced) return fail(RobotErrc::not_synced, "");
    if (s->robot_state != State::Stop)
        return fail(RobotErrc::not_stopped, std::string("robot state is '") + to_string(s->robot_state) + "', needs 'stop'");
    return {};
}

asio::awaitable<Result<std::string>> Robot::version() {
    const Reply r = co_await rpc_.call("getSoftVersion");
    if (r.ec) co_return Result<std::string>{fail(r.ec, "getSoftVersion: " + r.ec.message()), {}};
    co_return Result<std::string>{{}, r.value.is_string() ? r.value.get<std::string>() : r.value.dump()};
}

// --- power / setup ----------------------------------------------------------------------------------
asio::awaitable<std::error_code> Robot::power_on(std::chrono::milliseconds timeout) {
    if (std::error_code ec = require_remote()) co_return ec;

    if (status()->robot_state != State::Stop) {
        if (std::error_code ec = co_await command("clearAlarm")) co_return ec;
        const auto w = co_await monitor_.wait_until([](const RobotStatus& s) { return s.robot_state == State::Stop; },
                                                    timeout);
        if (w.first) co_return fail(RobotErrc::alarm_not_cleared, "alarm not cleared (" + describe() + "); check the pendant");
    }
    if (!status()->motor_synced) {
        // fails until the brakes are released on the pendant
        if (std::error_code ec = co_await command("syncMotorStatus")) co_return ec;
        const auto w = co_await monitor_.wait_until([](const RobotStatus& s) { return s.motor_synced; }, timeout);
        if (w.first) co_return fail(RobotErrc::sync_failed, "");
    }
    if (!status()->servo_ready) {
        nlohmann::json on = {{"status", 1}};
        if (std::error_code ec = co_await command("set_servo_status", on)) co_return ec;
        const auto w = co_await monitor_.wait_until([](const RobotStatus& s) { return s.servo_ready; }, timeout);
        if (w.first) co_return fail(RobotErrc::servos_did_not_start, "");
    }
    co_return std::error_code{};
}

asio::awaitable<std::error_code> Robot::power_off() {
    nlohmann::json off = {{"status", 0}};
    if (std::error_code ec = co_await command("set_servo_status", off)) co_return ec;
    const auto w = co_await monitor_.wait_until([](const RobotStatus& s) { return !s.servo_ready; },
                                                std::chrono::milliseconds(5000));
    co_return w.first ? fail(w.first, "servos did not switch off") : std::error_code{};
}

asio::awaitable<std::error_code> Robot::set_speed(double percent) {
    if (std::error_code ec = require_remote()) co_return ec;
    nlohmann::json p = {{"value", percent}};
    co_return co_await command("setSpeed", p);
}

asio::awaitable<std::error_code> Robot::set_tool(int tool_num) {
    if (std::error_code ec = require_remote()) co_return ec;
    nlohmann::json p = {{"tool_num", tool_num}};
    if (std::error_code ec = co_await command("setAutoRunToolNumber", p)) co_return ec;
    const Reply r = co_await rpc_.call("getAutoRunToolNumber");
    if (r.ec) co_return fail(r.ec, "getAutoRunToolNumber: " + r.ec.message());
    if (r.value != tool_num)
        co_return fail(RobotErrc::tool_not_selected, "tool " + std::to_string(tool_num) + " was not selected");
    co_return std::error_code{};
}

asio::awaitable<Result<ToolInfo>> Robot::tool_info(int tool_num) {
    Result<ToolInfo> out;
    nlohmann::json tcp_params = {{"tool_num", tool_num}, {"unit_type", 1}};
    const Reply tcp = co_await rpc_.call("getTcpPos", tcp_params);
    nlohmann::json tool_params = {{"tool_num", tool_num}};
    const Reply mass = co_await rpc_.call("getPayload", tool_params);
    const Reply cog = co_await rpc_.call("getCentreMass", tool_params);
    for (const Reply* r : {&tcp, &mass, &cog}) {
        if (r->ec) {
            out.ec = fail(r->ec, "tool info: " + r->ec.message());
            co_return out;
        }
    }
    to_six(tcp.value, out.value.tcp);
    if (mass.value.is_number()) out.value.mass = mass.value.get<double>();
    for (int i = 0; i < 3 && cog.value.is_array() && i < static_cast<int>(cog.value.size()); ++i)
        out.value.cog[i] = cog.value[i].get<double>();
    co_return out;
}

asio::awaitable<std::error_code> Robot::stop() { co_return co_await command("stop"); }

asio::awaitable<std::error_code> Robot::clear_alarm() { co_return co_await command("clearAlarm"); }

asio::awaitable<std::error_code> Robot::stop_trajectory() { co_return co_await command("stop_trajectory"); }

// --- motion -----------------------------------------------------------------------------------------
asio::awaitable<std::error_code> Robot::wait_until_stopped(std::chrono::seconds timeout, std::optional<Joints> q_target,
                                                           double tol) {
    // Did the motion start at all? (A move to where the arm already is never leaves 'stop'.)
    const auto started = co_await monitor_.wait_until(
        [](const RobotStatus& s) { return s.robot_state != State::Stop; }, std::chrono::milliseconds(500));
    if (started.first) {
        const auto s = status();
        if (q_target && s && max_abs_diff(s->q(), *q_target) <= tol) co_return std::error_code{};   // already there
        co_return fail(RobotErrc::motion_failed, "the motion never started (" + describe() + ")");
    }
    const auto ended = co_await monitor_.wait_until(
        [](const RobotStatus& s) { return s.robot_state != State::Running; },
        std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
    if (ended.first == asio::error::timed_out) {
        co_await stop();
        co_return fail(RobotErrc::motion_timeout, "");
    }
    if (ended.first) co_return fail(ended.first, "lost the robot state while moving");
    if (ended.second.robot_state != State::Stop)
        co_return fail(RobotErrc::motion_failed,
                       std::string("motion ended in state '") + to_string(ended.second.robot_state) + "'");
    co_return std::error_code{};
}

asio::awaitable<std::error_code> Robot::move_joint(Joints q, double speed, bool wait, std::chrono::seconds timeout) {
    if (std::error_code ec = require_ready()) co_return ec;
    // already there: nothing to send, and no 500 ms wait for a motion that would never start
    if (max_abs_diff(status()->q(), q) <= 0.01) co_return std::error_code{};
    nlohmann::json p = {{"targetPos", q}, {"speed", speed}};
    if (std::error_code ec = co_await command("moveByJoint", p)) co_return ec;
    if (!wait) co_return std::error_code{};
    co_return co_await wait_until_stopped(timeout, q);
}

asio::awaitable<Result<Joints>> Robot::plan_line(Pose pose, std::optional<double> max_joint_change) {
    const auto s = status();
    if (!s) co_return Result<Joints>{fail(RobotErrc::no_state, ""), {}};
    const Joints q_now = s->q();
    const Result<Joints> ik = co_await inverse_kinematics(pose, q_now);
    if (ik.ec) co_return ik;
    const double dist = std::sqrt(std::pow(pose[0] - s->tcp_pose[0], 2) + std::pow(pose[1] - s->tcp_pose[1], 2) +
                                  std::pow(pose[2] - s->tcp_pose[2], 2));
    const double limit = max_joint_change.value_or(5.0 + 0.5 * dist);
    const double change = max_abs_diff(q_now, ik.value);
    if (change > limit) {
        char text[160];
        std::snprintf(text, sizeof text, "IK solution changes a joint by %.1f deg for a %.1f mm move (limit %.1f)",
                      change, dist, limit);
        co_return Result<Joints>{fail(RobotErrc::ik_jump, text), ik.value};
    }
    co_return ik;
}

asio::awaitable<Result<Joints>> Robot::move_line(Pose pose, double speed, bool wait,
                                                 std::optional<double> max_joint_change, std::chrono::seconds timeout) {
    const Result<Joints> plan = co_await plan_line(pose, max_joint_change);
    if (plan.ec) co_return plan;
    if (std::error_code ec = require_ready()) co_return Result<Joints>{ec, plan.value};
    // joint form only: the target_pose form misbehaved on V3.11.2
    nlohmann::json p = {{"targetPos", plan.value}, {"speed", speed}};
    if (std::error_code ec = co_await command("moveByLine", p)) co_return Result<Joints>{ec, plan.value};
    if (!wait) co_return plan;
    const std::error_code ec = co_await wait_until_stopped(timeout, plan.value);
    co_return Result<Joints>{ec, plan.value};
}

// --- trajectory -----------------------------------------------------------------------------------
asio::awaitable<void> Robot::push_one(nlohmann::json params, Upload& upload) {
    const Reply r = co_await rpc_.call("push_pos", params);
    if (!upload.first_error) {
        if (r.ec) upload.first_error = r.ec;
        else if (r.value == false) upload.first_error = make_error_code(RobotErrc::rejected);
    }
    --upload.in_flight;
    upload.wake.cancel();   // lets wait_for_slot look again
}

asio::awaitable<void> Robot::wait_for_slot(Upload& upload, int max_in_flight) {
    while (upload.in_flight >= max_in_flight) {
        upload.wake.expires_after(std::chrono::seconds(10));   // a request finishing cancels this
        std::error_code ignored;
        co_await upload.wake.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
    }
}

asio::awaitable<std::error_code> Robot::run_joint_trajectory(std::vector<double> times, std::vector<Joints> points,
                                                             double speed_percent, double approach_speed, bool wait,
                                                             std::chrono::seconds timeout, const bool* cancel,
                                                             int upload_window) {
    auto cancelled = [cancel] { return cancel && *cancel; };
    if (times.size() != points.size() || points.size() < 3)
        co_return fail(RobotErrc::bad_trajectory, "need >= 3 points and one timestamp per point");
    if (times.front() != 0.0) co_return fail(RobotErrc::bad_trajectory, "timestamps must start at 0");
    for (std::size_t i = 1; i < times.size(); ++i)
        if (times[i] <= times[i - 1]) co_return fail(RobotErrc::bad_trajectory, "timestamps must increase");
    if (cancelled()) co_return fail(RobotErrc::cancelled, "trajectory cancelled");

    if (std::error_code ec = co_await move_joint(points.front(), approach_speed)) co_return ec;
    if (std::error_code ec = co_await command("flush_trajectory")) co_return ec;
    nlohmann::json header = {{"path_lenth", points.size()},   // 'lenth' as in the manual
                             {"pos_type", 0},
                             {"ref_joint_pos", points.front()},
                             {"ref_frame", Pose{}},
                             {"ret_flag", 1}};
    if (std::error_code ec = co_await command("start_push_pos", header)) co_return ec;

    // Upload with up to `upload_window` requests in flight. The writer queue keeps them in order.
    Upload upload(context_);
    const int window = std::max(1, upload_window);
    for (std::size_t i = 0; i < points.size() && !upload.first_error && !cancelled(); ++i) {
        co_await wait_for_slot(upload, window);
        nlohmann::json p = {{"timestamp", times[i]}, {"pos", points[i]}};
        ++upload.in_flight;
        asio::co_spawn(context_, push_one(std::move(p), upload), asio::detached);
    }
    co_await wait_for_slot(upload, 1);   // all replies in: `upload` must outlive every push_one

    if (cancelled() || upload.first_error) {
        co_await command("stop_push_pos");
        co_await command("flush_trajectory");
        if (cancelled()) co_return fail(RobotErrc::cancelled, "trajectory cancelled during upload");
        co_return fail(upload.first_error, "push_pos: " + upload.first_error.message());
    }
    if (std::error_code ec = co_await command("stop_push_pos"))
        co_return fail(ec, "stop_push_pos: not all points were accepted");
    const Reply check = co_await rpc_.call("check_trajectory");
    if (check.ec) co_return fail(check.ec, "check_trajectory: " + check.ec.message());
    const int code = check.value.is_number_integer() ? check.value.get<int>() : -99;
    if (code != 0) {
        const char* reason = code == -1 ? "point count mismatch" : code == -2 ? "point format error"
                           : code == -3 ? "timestamps invalid" : "unknown";
        co_return fail(RobotErrc::trajectory_rejected, std::string("trajectory rejected: ") + reason);
    }
    if (cancelled()) {
        co_await command("flush_trajectory");
        co_return fail(RobotErrc::cancelled, "trajectory cancelled before start");
    }
    nlohmann::json start = {{"speed_percent", speed_percent}};
    if (std::error_code ec = co_await command("start_trajectory", start)) co_return ec;
    if (!wait) co_return std::error_code{};
    co_return co_await wait_until_stopped(timeout, points.back());
}

// --- kinematics -------------------------------------------------------------------------------------
asio::awaitable<Result<Pose>> Robot::forward_kinematics(Joints q) {
    nlohmann::json p = {{"targetPos", q}, {"unit_type", 1}};
    const Reply r = co_await rpc_.call("positiveKinematic", p);
    Result<Pose> out;
    if (r.ec) out.ec = fail(r.ec, "positiveKinematic: " + r.ec.message());
    else if (!to_six(r.value, out.value)) out.ec = fail(RobotErrc::rejected, "positiveKinematic: unexpected reply " + r.value.dump());
    co_return out;
}

asio::awaitable<Result<Joints>> Robot::inverse_kinematics(Pose pose, std::optional<Joints> ref_q) {
    nlohmann::json p = {{"targetPose", pose}, {"unit_type", 1}};
    if (ref_q) p["referencePos"] = *ref_q;
    const Reply r = co_await rpc_.call("inverseKinematic", p);
    Result<Joints> out;
    if (r.ec) out.ec = fail(r.ec, "inverseKinematic: " + r.ec.message());
    else if (!to_six(r.value, out.value)) out.ec = fail(RobotErrc::rejected, "inverseKinematic: unexpected reply " + r.value.dump());
    co_return out;
}

}  // namespace ec
