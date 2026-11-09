#include "ec_robot/streamer.hpp"

#include <algorithm>
#include <cmath>

#include "ec_robot/robot.hpp"

namespace ec {

JointStreamer::JointStreamer(Robot& robot, StreamerConfig config) : robot_(robot), cfg_(config) {
    if (cfg_.period_ms < 2 || cfg_.period_ms > 100) throw std::invalid_argument("period_ms must be within 2..100");
    if (cfg_.lookahead_ms < 10 || cfg_.lookahead_ms > 1000)
        throw std::invalid_argument("lookahead_ms must be within 10..1000");
    if (cfg_.smoothness < 0.0 || cfg_.smoothness > 1.0) throw std::invalid_argument("smoothness must be within 0..1");
    period_ = std::chrono::milliseconds(cfg_.period_ms);
    max_step_ = cfg_.max_joint_speed * cfg_.period_ms / 1000.0;
}

JointStreamer::~JointStreamer() {
    try {
        if (running()) stop(/*settle=*/false);
    } catch (...) {
    }
}

void JointStreamer::start() {
    robot_.require_ready();
    auto& rpc = robot_.rpc();
    if (rpc.call("get_transparent_transmission_state") == 1) {
        rpc.call("tt_clear_servo_joint_buf");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));   // as in the manual's examples
    }
    const Joints q0 = robot_.status().q();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = target_ = q0;
        seg_start_.reset();
        seg_dt_ = cfg_.period_ms / 1000.0;
        last_set_ = {};
        error_.reset();
    }
    auto ok = rpc.call("transparent_transmission_init", {{"lookahead", cfg_.lookahead_ms},
                                                          {"t", cfg_.period_ms},
                                                          {"smoothness", cfg_.smoothness},
                                                          {"response_enable", 0}});
    if (ok == false) throw StreamError("transparent_transmission_init was rejected");
    stopping_ = false;
    ticks_ = 0;
    overruns_ = 0;
    thread_ = std::thread([this] { run(); });
}

void JointStreamer::set_target(const Joints& q_in) {
    if (auto e = error()) throw StreamError(*e);
    if (!running()) throw StreamError("streamer is not running");
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = std::clamp(q_in[i], cfg_.joint_min[i], cfg_.joint_max[i]);
    seg_start_ = interp(now);                     // continue from where we are now
    seg_t0_ = now;
    const double gap = last_set_.time_since_epoch().count() == 0
                           ? cfg_.period_ms / 1000.0
                           : std::chrono::duration<double>(now - last_set_).count();
    seg_dt_ = std::clamp(gap, cfg_.period_ms / 1000.0, 0.5);
    target_ = q;
    last_set_ = now;
}

void JointStreamer::set_target_delta(const Joints& dq) {
    Joints base;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        base = target_;
    }
    for (int i = 0; i < 6; ++i) base[i] += dq[i];
    set_target(base);
}

Joints JointStreamer::interp(Clock::time_point now) const {
    if (!seg_start_) return target_;
    const double a = std::min(1.0, std::chrono::duration<double>(now - seg_t0_).count() / seg_dt_);
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = (*seg_start_)[i] + a * (target_[i] - (*seg_start_)[i]);
    return q;
}

Joints JointStreamer::command() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cmd_;
}

bool JointStreamer::at_target(double tol) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 6; ++i) {
        if (std::abs(cmd_[i] - target_[i]) > tol) return false;
    }
    return true;
}

std::optional<std::string> JointStreamer::error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}

void JointStreamer::stop(bool settle) {
    if (!running()) return;
    if (settle && !error()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_ = cmd_;          // stop moving the setpoint
            seg_start_.reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.lookahead_ms + 200));   // drain lookahead
    }
    stopping_ = true;
    thread_.join();
    try {
        robot_.rpc().call("tt_clear_servo_joint_buf");
    } catch (const ECError&) {
    }
    // don't return until the robot has left "running", so the next command sees fresh state
    robot_.monitor().wait_for([](const RobotStatus& s) { return s.robot_state != State::Running; },
                              std::chrono::milliseconds(2000));
    if (auto e = error()) throw StreamError(*e);
}

void JointStreamer::run() {
    auto& rpc = robot_.rpc();
    auto& mon = robot_.monitor();
    auto next_tick = Clock::now();
    try {
        while (!stopping_) {
            Joints cmd;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const Joints goal = interp(Clock::now());
                for (int i = 0; i < 6; ++i) {
                    const double d = goal[i] - cmd_[i];
                    cmd_[i] += std::clamp(d, -max_step_, max_step_);
                }
                cmd = cmd_;
            }
            rpc.notify("tt_put_servo_joint_to_buf", {{"targetPos", cmd}});
            ++ticks_;

            // safety checks against live monitor data
            const RobotStatus s = mon.latest(std::chrono::milliseconds(100));
            if (s.robot_state == State::EStop || s.robot_state == State::Alarm ||
                s.robot_state == State::Collision) {
                throw StreamError(std::string("robot entered state '") + to_string(s.robot_state) + "'");
            }
            if (s.robot_mode != Mode::Remote) throw StreamError("robot left remote mode");
            double err = 0;
            const Joints q = s.q();
            for (int i = 0; i < 6; ++i) err = std::max(err, std::abs(q[i] - cmd[i]));
            if (err > cfg_.max_tracking_error) {
                throw StreamError("tracking error " + std::to_string(err) + " deg exceeds " +
                                  std::to_string(cfg_.max_tracking_error) + " deg");
            }

            // fixed-rate schedule without drift
            next_tick += period_;
            const auto now = Clock::now();
            if (next_tick > now) {
                std::this_thread::sleep_until(next_tick);
            } else if (now - next_tick > period_) {
                ++overruns_;
                next_tick = now;
            }
        }
    } catch (const std::exception& e) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = e.what();
        }
        try {
            rpc.call("stop");
        } catch (...) {
        }
    }
}

}  // namespace ec
