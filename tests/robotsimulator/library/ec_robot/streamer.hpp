// Joint streaming through the transparent transmission service (manual 2.2.7), async.
//
// One coroutine on the robot's io_context sends a joint setpoint every `period` with
// tt_put_servo_joint_to_buf, without waiting for replies (response_enable = 0), on a fixed
// schedule: timer.expires_at(timer.expiry() + period).
// Callers only set targets, at any rate (a policy at 5-50 Hz, teleop, a script). The
// setpoint moves linearly to each new target over the interval since the previous one,
// then a per-joint speed limit applies, so a slow caller never starves the controller and
// a bad target can't make the arm jump.
//
// Every tick the live monitor data is checked; the stream aborts and sends `stop` on
// e-stop / alarm / collision, leaving remote mode, stale monitor data (> 100 ms), or a
// tracking error above the limit. error() then says why.
//
// Threads: everything, set_target included, runs on the robot's io_context thread. A policy
// on another thread posts its targets: asio::post(context, [&, q] { streamer.set_target(q); }).
// The streamer must outlive its loop: call co_await stop() before destroying it.
//
// The 8 ms deadline is real: this io_context must not also render or plan, or a long
// frame/plan starves the controller's buffer (the lookahead only covers 200 ms).
//
// Measured on controller V3.11.2: lookahead 100 ms vibrates even with perfect setpoints,
// 200 ms is smooth, hence the 200 ms default.
#pragma once

#include <utility>   // before asio.hpp: Asio 1.18.1 uses std::exchange without including it
#include "asio.hpp"

#include <cstdint>
#include <optional>
#include <string>

#include "robot.hpp"

namespace ec {

struct StreamerConfig {
    int period_ms = 8;                // setpoint period t, 2..100 ms
    int lookahead_ms = 200;           // controller lookahead, 10..1000 ms
    double smoothness = 0.5;          // controller gain, 0..1
    double max_joint_speed = 30.0;    // deg/s, applied to every joint
    double max_tracking_error = 10.0; // deg
    Joints joint_min{-360, -360, -360, -360, -360, -360};
    Joints joint_max{360, 360, 360, 360, 360, 360};
};

enum class StreamErrc {
    bad_config = 1,     // a StreamerConfig value outside its range
    not_running,
    init_rejected,      // transparent_transmission_init answered false
    robot_fault,        // e-stop, alarm or collision
    left_remote,
    stale_state,        // no monitor packet for > 100 ms
    tracking_error,     // measured joints too far from the setpoint
};
std::error_code make_error_code(StreamErrc e);

class Streamer {
public:
    Streamer(Robot& robot, StreamerConfig config = {});

    asio::awaitable<std::error_code> start();
    // Hold the last setpoint, let the lookahead window drain (settle), end the loop, clear the
    // controller's buffer and wait until the robot leaves "running". Returns why the stream had
    // aborted, if it had; otherwise ok.
    asio::awaitable<std::error_code> stop(bool settle = true);

    std::error_code set_target(const Joints& q);          // deg; returns immediately
    std::error_code set_target_delta(const Joints& dq);   // relative to the current target

    Joints command() const { return cmd_; }               // setpoint most recently sent
    bool at_target(double tol = 0.05) const;
    bool running() const { return running_; }
    std::error_code error() const { return error_; }      // why it aborted (empty: it didn't)
    const std::string& error_detail() const { return detail_; }
    std::uint64_t ticks() const { return ticks_; }
    std::uint64_t overruns() const { return overruns_; }

private:
    asio::awaitable<void> run();
    Joints interp(Clock::time_point now) const;
    std::error_code abort_check(const Joints& cmd);   // safety checks on the latest packet

    Robot& robot_;
    asio::io_context& context_;
    StreamerConfig cfg_;
    double max_step_ = 0;                  // deg per tick
    std::chrono::milliseconds period_;

    Joints target_{};
    Joints cmd_{};
    std::optional<Joints> seg_start_;      // interpolation segment towards target_
    Clock::time_point seg_t0_{};
    double seg_dt_ = 0.008;
    Clock::time_point last_set_{};

    asio::steady_timer tick_;              // the fixed-rate schedule
    asio::steady_timer loop_done_;         // cancelled when run() returns: wakes stop()
    bool running_ = false;
    bool stopping_ = false;
    std::error_code error_;
    std::string detail_;
    std::uint64_t ticks_ = 0, overruns_ = 0;
};

}  // namespace ec

template <>
struct std::is_error_code_enum<ec::StreamErrc> : std::true_type {};