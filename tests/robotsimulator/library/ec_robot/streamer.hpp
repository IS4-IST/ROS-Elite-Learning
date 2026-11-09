// Joint streaming through the transparent transmission service (manual 2.2.7).
//
// A dedicated thread sends one joint setpoint every `period` with
// tt_put_servo_joint_to_buf, without waiting for replies (response_enable = 0).
// Callers only set targets, at any rate (a policy at 5-50 Hz, teleop, a script).
// The setpoint moves linearly to each new target over the interval since the
// previous one, then a per-joint speed limit applies, so a slow caller never
// starves the controller and a bad target can't make the arm jump.
//
// Every tick the live monitor data is checked; the stream aborts and sends
// `stop` on e-stop / alarm / collision, leaving remote mode, stale monitor data,
// or a tracking error above the limit.
//
// Measured on controller V3.11.2: lookahead 100 ms vibrates even with perfect
// setpoints, 200 ms is smooth, hence the 200 ms default.
#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "ec_robot/types.hpp"

namespace ec {

class Robot;

struct StreamerConfig {
    int period_ms = 8;                // setpoint period t, 2..100 ms
    int lookahead_ms = 200;           // controller lookahead, 10..1000 ms
    double smoothness = 0.5;          // controller gain, 0..1
    double max_joint_speed = 30.0;    // deg/s, applied to every joint
    double max_tracking_error = 10.0; // deg
    Joints joint_min{-360, -360, -360, -360, -360, -360};
    Joints joint_max{360, 360, 360, 360, 360, 360};
};

class StreamError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class JointStreamer {
public:
    JointStreamer(Robot& robot, StreamerConfig config = {});
    ~JointStreamer();
    JointStreamer(const JointStreamer&) = delete;
    JointStreamer& operator=(const JointStreamer&) = delete;

    void start();
    // Hold the last setpoint, let the lookahead window drain, clear the buffer.
    // Throws StreamError if the stream had aborted.
    void stop(bool settle = true);

    void set_target(const Joints& q);          // deg; returns immediately
    void set_target_delta(const Joints& dq);   // relative to the current target
    Joints command() const;                    // setpoint most recently sent
    bool at_target(double tol = 0.05) const;
    bool running() const { return thread_.joinable(); }
    std::optional<std::string> error() const;

    std::uint64_t ticks() const { return ticks_; }
    std::uint64_t overruns() const { return overruns_; }

private:
    void run();
    Joints interp(Clock::time_point now) const;   // lock held

    Robot& robot_;
    StreamerConfig cfg_;
    double max_step_;
    std::chrono::nanoseconds period_;

    mutable std::mutex mutex_;
    Joints target_{};
    Joints cmd_{};
    std::optional<Joints> seg_start_;
    Clock::time_point seg_t0_{};
    double seg_dt_ = 0.008;
    Clock::time_point last_set_{};
    std::optional<std::string> error_;

    std::thread thread_;
    std::atomic<bool> stopping_{false};
    std::atomic<std::uint64_t> ticks_{0};
    std::atomic<std::uint64_t> overruns_{0};
};

}  // namespace ec
