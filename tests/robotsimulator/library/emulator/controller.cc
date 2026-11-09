#include "emulator/controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>

namespace emu {
namespace {

constexpr double kDeg = M_PI / 180.0;
const std::set<std::string> kRemoteOnly = {
    "set_servo_status", "syncMotorStatus", "clearAlarm", "stop", "setSpeed", "moveByJoint", "moveByLine",
    "moveBySpeedj", "stopj", "setAutoRunToolNumber", "cmd_set_tcp", "cmd_set_payload",
    "transparent_transmission_init", "tt_put_servo_joint_to_buf", "tt_clear_servo_joint_buf",
    "start_trajectory", "pause_trajectory", "resume_trajectory", "stop_trajectory"};

Joints joints_from(const nlohmann::json& j, const char* name) {
    if (!j.is_array() || j.size() < 6) throw RpcError{-32602, std::string(name) + " must have 6 values"};
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = j[i].get<double>();
    return q;
}

}  // namespace

Controller::Controller(mjModel* m, mjData* d, ControllerOptions opts) : m_(m), d_(d), opts_(opts), kin_(m) {
    if (opts_.cold_start) {
        mode_ = 0;
        brakes_released_ = false;
        synced_ = false;
    } else {
        mode_ = 2;
        brakes_released_ = true;
        synced_ = true;
    }
    tools_[1] = Tool{Pose{}, 0.3, {0, 0, 100}};   // like the lab robot's tool 1
    cmd_ = measured();
    brake_q_ = cmd_;
    last_q_ = cmd_;
    last_tcp_ = kin_.forward(cmd_, tool());
}

Joints Controller::measured() const {
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = d_->qpos[i] / kDeg;
    return q;
}

std::string Controller::describe() {
    static const char* modes[] = {"teach", "play", "remote"};
    static const char* states[] = {"stop", "pause", "e-stop", "running", "alarm", "collision"};
    std::lock_guard<std::mutex> lock(mutex_);
    return std::string("mode=") + modes[mode_] + " state=" + states[state_] + " servo=" + (servo_ ? "on" : "off") +
           " synced=" + (synced_ ? "yes" : "no") + " brakes=" + (brakes_released_ ? "released" : "closed") +
           " tool=" + std::to_string(tool_);
}

Controller::PendantView Controller::pendant() {
    std::lock_guard<std::mutex> lock(mutex_);
    PendantView v;
    v.mode = mode_;
    v.state = state_;
    v.servo = servo_;
    v.synced = synced_;
    v.brakes_released = brakes_released_;
    v.estop = estop_;
    v.tool = tool_;
    v.speed_override = speed_override_;
    return v;
}

// --- pendant actions ---------------------------------------------------------
void Controller::toggle_mode() {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode_ == 2 ? 0 : 2;
    if (motion_) cancel_motion();          // leaving remote stops remote motion
    if (state_ == 3) state_ = 0;
}

void Controller::set_estop(bool pressed) {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_ = pressed;
    if (pressed) {
        cancel_motion();
        servo_ = false;
        brake_q_ = measured();
        state_ = 2;
    } else if (state_ == 2) {
        state_ = 4;                         // released: alarm until clearAlarm
    }
}

void Controller::release_brakes(bool released) {
    std::lock_guard<std::mutex> lock(mutex_);
    brakes_released_ = released;
    if (released) synced_ = true;           // observed on V3.11.2
    else if (servo_) { servo_ = false; brake_q_ = measured(); }
}

// --- motion bookkeeping ----------------------------------------------------------
void Controller::start_motion(std::unique_ptr<Motion> m) {
    cmd_ = measured();
    motion_ = std::move(m);
    pass_through_ = nullptr;
    playback_ = nullptr;
    state_ = 3;
}

void Controller::cancel_motion() {
    motion_.reset();
    pass_through_ = nullptr;
    playback_ = nullptr;
    cmd_ = measured();
    if (state_ == 3 || state_ == 1) state_ = 0;
}

void Controller::raise(int state, const std::string& /*why*/) {
    motion_.reset();
    pass_through_ = nullptr;
    playback_ = nullptr;
    cmd_ = measured();
    state_ = state;
}

void Controller::require_remote(const std::string& method) const {
    if (kRemoteOnly.count(method) && mode_ != 2)
        throw RpcError{-32000, "This command is only supported in remote mode"};
}

// --- JSON-RPC -------------------------------------------------------------------
std::optional<nlohmann::json> Controller::handle_request(const nlohmann::json& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto id = request.value("id", nlohmann::json());
    const std::string method = request.value("method", std::string());
    const nlohmann::json params = request.contains("params") ? request["params"] : nlohmann::json::object();
    nlohmann::json reply = {{"jsonrpc", "2.0"}, {"id", id}};
    bool send = true;
    try {
        nlohmann::json result = dispatch(method, params, send);
        reply["result"] = result.dump();    // results travel as JSON text
    } catch (const RpcError& e) {
        send = true;
        reply["error"] = {{"code", e.code}, {"message", e.message}};
    } catch (const nlohmann::json::exception& e) {
        send = true;
        reply["error"] = {{"code", -32602}, {"message", std::string("invalid params: ") + e.what()}};
    }
    if (!send) return std::nullopt;
    return reply;
}

nlohmann::json Controller::dispatch(const std::string& method, const nlohmann::json& p, bool& reply) {
    require_remote(method);
    auto ready_for_motion = [&] {
        if (!servo_ || !synced_ || state_ != 0) return false;
        return true;
    };

    // --- status ---
    if (method == "getSoftVersion") return opts_.version;
    if (method == "getRobotMode") return mode_;
    if (method == "getRobotState") return state_;
    if (method == "getServoStatus") return servo_;
    if (method == "getMotorStatus") return synced_;
    if (method == "get_estop_status") return estop_ ? 1 : 0;
    if (method == "getSpeed") return speed_override_;
    if (method == "get_servo_brake_off_status") return std::vector<int>(6, brakes_released_ ? 1 : 0);
    if (method == "get_joint_pos") return measured();
    if (method == "get_tcp_pose") {
        int t = p.value("tool_num", -1);
        const Pose tcp = t >= 0 && t < 8 ? tools_[t].tcp : tool();
        Pose pose = kin_.forward(measured(), tcp);
        if (p.value("unit_type", 1) == 0) for (int i = 3; i < 6; ++i) pose[i] /= kDeg;
        return pose;
    }

    // --- power ---
    if (method == "clearAlarm") {
        if (state_ == 4 || state_ == 5 || (state_ == 2 && !estop_)) state_ = 0;
        return true;
    }
    if (method == "syncMotorStatus") {
        if (!brakes_released_) throw RpcError{-32000, "Failed to sync motor status, the brake of robot is not opened."};
        synced_ = true;
        return true;
    }
    if (method == "set_servo_status") {
        const int on = p.at("status").get<int>();
        if (on) {
            if (estop_ || !synced_ || !brakes_released_) return false;
            servo_ = true;
            cmd_ = brake_q_;
        } else {
            cancel_motion();
            servo_ = false;
            brake_q_ = measured();
        }
        return true;
    }
    if (method == "setSpeed") {
        speed_override_ = std::clamp(p.at("value").get<double>(), 0.05, 100.0);
        return true;
    }
    if (method == "stop") {
        if (motion_) cancel_motion();
        return true;
    }

    // --- tools ---
    if (method == "getToolNumber" || method == "getAutoRunToolNumber") return tool_;
    if (method == "setAutoRunToolNumber" || method == "setToolNumber") {
        const int t = p.at("tool_num").get<int>();
        if (t < 0 || t > 7) throw RpcError{-32602, "tool_num must be 0..7"};
        tool_ = t;
        return true;
    }
    if (method == "getTcpPos") {
        Pose t = tools_.at(p.at("tool_num").get<int>()).tcp;
        if (p.value("unit_type", 1) == 0) for (int i = 3; i < 6; ++i) t[i] /= kDeg;
        return t;
    }
    if (method == "getPayload") return tools_.at(p.at("tool_num").get<int>()).mass;
    if (method == "getCentreMass") return tools_.at(p.at("tool_num").get<int>()).cog;
    if (method == "cmd_set_tcp") {
        Pose t = joints_from(p.at("point"), "point");
        if (p.value("unit_type", 1) == 0) for (int i = 3; i < 6; ++i) t[i] *= kDeg;
        tools_.at(p.at("tool_num").get<int>()).tcp = t;
        return true;
    }
    if (method == "cmd_set_payload") {
        auto& t = tools_.at(p.at("tool_num").get<int>());
        t.mass = p.at("m").get<double>();
        const auto& c = p.contains("cog") ? p["cog"] : p.value("point", nlohmann::json::array({0, 0, 0}));
        for (int i = 0; i < 3; ++i) t.cog[i] = c.at(i).get<double>();
        return true;
    }

    // --- kinematics ---
    if (method == "positiveKinematic") {
        Pose pose = kin_.forward(joints_from(p.at("targetPos"), "targetPos"), tool());
        if (p.value("unit_type", 1) == 0) for (int i = 3; i < 6; ++i) pose[i] /= kDeg;
        return pose;
    }
    if (method == "inverseKinematic") {
        Pose pose = joints_from(p.at("targetPose"), "targetPose");
        if (p.value("unit_type", 1) == 0) for (int i = 3; i < 6; ++i) pose[i] *= kDeg;
        const Joints ref = p.contains("referencePos") ? joints_from(p["referencePos"], "referencePos") : measured();
        auto q = kin_.inverse(pose, tool(), ref);
        if (!q) throw RpcError{-32000, "inverse kinematics failed"};
        return *q;
    }

    // --- point-to-point ---
    if (method == "moveByJoint") {
        if (!p.contains("targetPos")) throw RpcError{-32602, "moveByJoint needs targetPos (joint form)"};
        const Joints target = joints_from(p["targetPos"], "targetPos");
        if (!kin_.within_limits(target)) throw RpcError{-32000, "target outside the joint limits"};
        if (!ready_for_motion()) return false;
        const double v = opts_.max_joint_speed * std::clamp(p.value("speed", 10.0), 0.01, 100.0) / 100.0 *
                         speed_override_ / 100.0;
        start_motion(std::make_unique<JointMove>(measured(), target, v, opts_.max_joint_acc));
        return true;
    }
    if (method == "moveByLine") {
        if (p.contains("target_pose") && !p.contains("targetPos"))
            throw RpcError{-32602, "target_pose is not supported (it misbehaved on V3.11.2); send targetPos"};
        const Joints target = joints_from(p.at("targetPos"), "targetPos");
        if (!kin_.within_limits(target)) throw RpcError{-32000, "target outside the joint limits"};
        if (!ready_for_motion()) return false;
        const double speed = std::max(1.0, p.value("speed", 50.0)) * speed_override_ / 100.0;
        start_motion(std::make_unique<LineMove>(kin_, tool(), measured(), target, speed));
        return true;
    }
    if (method == "moveBySpeedj") {
        if (!ready_for_motion() && !(state_ == 3 && dynamic_cast<SpeedJ*>(motion_.get()))) return false;
        const Joints v = joints_from(p.at("vj"), "vj");
        start_motion(std::make_unique<SpeedJ>(v, p.value("acc", 30.0), p.value("t", 0.1)));
        return true;
    }
    if (method == "stopj") {
        if (auto* s = dynamic_cast<SpeedJ*>(motion_.get())) s->stop(p.value("acc", 30.0));
        return true;
    }

    // --- transparent transmission (2.2.7) ---
    if (method == "get_transparent_transmission_state") return pass_through_ ? 1 : 0;
    if (method == "transparent_transmission_init") {
        const int t = p.at("t").get<int>(), la = p.at("lookahead").get<int>();
        const double g = p.at("smoothness").get<double>();
        if (t < 2 || t > 100 || la < 10 || la > 1000 || g < 0 || g > 1)
            throw RpcError{-32602, "t 2..100, lookahead 10..1000, smoothness 0..1"};
        if (!ready_for_motion()) return false;
        auto pt = std::make_unique<PassThrough>(measured(), t / 1000.0, la / 1000.0, g);
        PassThrough* raw = pt.get();
        start_motion(std::move(pt));
        pass_through_ = raw;
        tt_reply_ = p.value("response_enable", 1) != 0;
        return true;
    }
    if (method == "tt_put_servo_joint_to_buf") {
        reply = tt_reply_;
        if (!pass_through_) {
            reply = true;
            return false;
        }
        if (!p.contains("targetPos")) throw RpcError{-32602, "only targetPos (joint form) is supported"};
        pass_through_->push(joints_from(p["targetPos"], "targetPos"));
        return true;
    }
    if (method == "tt_set_current_servo_joint") throw RpcError{-32000, "deprecated; use tt_put_servo_joint_to_buf"};
    if (method == "tt_clear_servo_joint_buf") {
        if (pass_through_) {
            // drop the buffer but let the arm settle on the last commanded point;
            // the state stays "running" until it is in position
            motion_.reset();
            pass_through_ = nullptr;
            settling_ = 0;
        }
        return true;
    }

    // --- trajectory service (2.2.9) ---
    if (method == "flush_trajectory") {
        upload_.reset();
        loaded_.reset();
        return true;
    }
    if (method == "start_push_pos") {
        if (p.value("pos_type", 0) != 0) throw RpcError{-32602, "only pos_type 0 (joints) is supported"};
        Upload u;
        u.expected = p.at("path_lenth").get<std::size_t>();   // 'lenth' as in the manual
        u.reply = p.value("ret_flag", 1) != 0;
        upload_ = std::move(u);
        return true;
    }
    if (method == "push_pos") {
        if (!upload_) throw RpcError{-32000, "start_push_pos first"};
        reply = upload_->reply;
        upload_->times.push_back(p.at("timestamp").get<double>());
        upload_->points.push_back(joints_from(p.at("pos"), "pos"));
        return true;
    }
    if (method == "stop_push_pos") return upload_ && upload_->points.size() == upload_->expected;
    if (method == "check_trajectory") {
        if (!upload_ || upload_->points.size() != upload_->expected) return -1;
        if (upload_->points.size() < 3) return -1;
        const auto& t = upload_->times;
        if (t.front() != 0.0) return -3;
        for (std::size_t i = 1; i < t.size(); ++i) if (t[i] <= t[i - 1]) return -3;
        for (const auto& q : upload_->points) if (!kin_.within_limits(q)) return -2;
        loaded_ = upload_;
        return 0;
    }
    if (method == "start_trajectory") {
        if (!loaded_ || !ready_for_motion()) return false;
        auto pb = std::make_unique<TrajectoryPlayback>(loaded_->times, loaded_->points,
                                                       p.value("speed_percent", 100.0));
        TrajectoryPlayback* raw = pb.get();
        start_motion(std::move(pb));
        playback_ = raw;
        return true;
    }
    if (method == "pause_trajectory") {
        if (playback_) { playback_->paused = true; state_ = 1; }
        return true;
    }
    if (method == "resume_trajectory") {
        if (playback_) { playback_->paused = false; state_ = 3; }
        return true;
    }
    if (method == "stop_trajectory") {
        if (playback_) cancel_motion();
        return true;
    }

    throw RpcError{-32601, "Method not found"};
}

// --- control tick ------------------------------------------------------------------
void Controller::check_collisions() {
    for (int i = 0; i < d_->ncon; ++i) {
        const mjContact& c = d_->contact[i];
        if (c.dist > -0.002) continue;               // ignore touching, react to > 2 mm penetration
        const int b1 = m_->geom_bodyid[c.geom[0]], b2 = m_->geom_bodyid[c.geom[1]];
        if (b1 > 1 || b2 > 1) {                      // involves a moving robot link
            raise(5, "collision");
            return;
        }
    }
}

void Controller::tick(double dt, double sim_time_target) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_tick_dt_ = dt;

    if (motion_ && servo_) {
        const bool more = motion_->step(dt, cmd_);
        if (!motion_->error.empty()) {
            raise(4, motion_->error);
        } else if (!more) {
            motion_.reset();
            pass_through_ = nullptr;
            playback_ = nullptr;
            settling_ = 0;                           // stay "running" until the arm is in position
        }
    }
    if (!servo_) cmd_ = brake_q_;                    // brakes hold the arm rigidly where it stopped

    // drive the position servos (rad) and catch physics up with wall time.
    // Gravity and Coriolis forces are compensated, as the real controller does,
    // so the servos only correct the remaining error and the arm ends on target.
    for (int i = 0; i < 6 && i < m_->nu; ++i) d_->ctrl[i] = cmd_[i] * kDeg;
    int steps = 0;
    while (d_->time < sim_time_target && steps < 50) {
        for (int i = 0; i < m_->nv; ++i) d_->qfrc_applied[i] = d_->qfrc_bias[i];
        mj_step(m_, d_);
        ++steps;
    }
    if (d_->time < sim_time_target - 0.1) d_->time = sim_time_target;   // don't spiral if we fell far behind

    if (opts_.collision_alarm && servo_) check_collisions();

    // in-position check after a finished motion: report "stop" once the arm has arrived
    if (state_ == 3 && !motion_) {
        settling_ += dt;
        double err = 0, vel = 0;
        for (int i = 0; i < 6; ++i) {
            err = std::max(err, std::abs(d_->qpos[i] / kDeg - cmd_[i]));
            vel = std::max(vel, std::abs(d_->qvel[i] / kDeg));
        }
        if (err < 0.01 && vel < 0.2) state_ = 0;
        else if (settling_ > 2.0) raise(4, "position not reached");   // like a following-error alarm
    }

    const Joints q = measured();
    Joints qd{};
    for (int i = 0; i < 6; ++i) qd[i] = d_->qvel[i] / kDeg;
    last_qd_ = qd;
    last_q_ = q;
    const Pose tcp = kin_.forward(q, tool());
    tcp_speed_ = std::sqrt(std::pow(tcp[0] - last_tcp_[0], 2) + std::pow(tcp[1] - last_tcp_[1], 2) +
                           std::pow(tcp[2] - last_tcp_[2], 2)) / std::max(dt, 1e-6);
    last_tcp_ = tcp;
}

std::vector<std::uint8_t> Controller::status_packet() {
    std::lock_guard<std::mutex> lock(mutex_);
    StatusFields f;
    f.timestamp_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
    f.joint_pos = last_q_;
    f.tcp_pose = last_tcp_;
    f.flange_pose = kin_.flange(last_q_);
    f.robot_state = state_;
    f.servo_ready = servo_;
    f.motor_synced = synced_;
    f.robot_mode = mode_;
    f.collision = state_ == 5;
    f.estop = estop_;
    f.tcp_speed = tcp_speed_;
    f.joint_speed = last_qd_;
    for (int i = 0; i < 6; ++i) {
        f.joint_temperature[i] = servo_ ? 30.0 : -40.0;              // like V3.11.2 with servos off
        f.joint_torque[i] = servo_ && i < m_->nv ? d_->qfrc_actuator[i] : 0.0;
    }
    return encode_status_packet(f);
}

}  // namespace emu
