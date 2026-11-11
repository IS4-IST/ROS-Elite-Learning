#include "streamer.h"

#include <algorithm>
#include <cmath>

// GCC 11-13 coroutine pitfalls, avoided here: no braced lists inside a co_await expression
// (build the json first), no structured bindings on a co_await result.

namespace ec {
namespace {

class StreamCategory : public std::error_category {
public:
    const char* name() const noexcept override { return "ec-stream"; }
    std::string message(int code) const override {
        switch (static_cast<StreamErrc>(code)) {
            case StreamErrc::bad_config: return "invalid streamer configuration";
            case StreamErrc::not_running: return "streamer is not running";
            case StreamErrc::init_rejected: return "transparent_transmission_init was rejected";
            case StreamErrc::robot_fault: return "robot entered e-stop, alarm or collision";
            case StreamErrc::left_remote: return "robot left remote mode";
            case StreamErrc::stale_state: return "monitor data older than 100 ms";
            case StreamErrc::tracking_error: return "tracking error above the limit";
        }
        return "unknown stream error";
    }
};

}  // namespace

std::error_code make_error_code(StreamErrc e) {
    static StreamCategory category;
    return {static_cast<int>(e), category};
}

Streamer::Streamer(Robot& robot, StreamerConfig config)
    : robot_(robot), context_(robot.context()), cfg_(config), period_(config.period_ms),
      tick_(robot.context()), loop_done_(robot.context()) {}

// --- start / stop -------------------------------------------------------------------------
asio::awaitable<std::error_code> Streamer::start() {
    if (running_) co_return std::error_code{};
    if (cfg_.period_ms < 2 || cfg_.period_ms > 100 || cfg_.lookahead_ms < 10 || cfg_.lookahead_ms > 1000 ||
        cfg_.smoothness < 0.0 || cfg_.smoothness > 1.0 || cfg_.max_joint_speed <= 0)
        co_return make_error_code(StreamErrc::bad_config);
    period_ = std::chrono::milliseconds(cfg_.period_ms);
    max_step_ = cfg_.max_joint_speed * cfg_.period_ms / 1000.0;
    if (std::error_code ec = robot_.require_ready()) co_return ec;

    // a stream left over from an earlier program: clear it first, as in the manual's examples
    const Reply state = co_await robot_.rpc().call("get_transparent_transmission_state");
    if (state.ec) co_return state.ec;
    if (state.value == 1) {
        co_await robot_.rpc().call("tt_clear_servo_joint_buf");
        asio::steady_timer pause(context_);
        pause.expires_after(std::chrono::milliseconds(500));
        std::error_code ignored;
        co_await pause.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
    }

    cmd_ = target_ = robot_.status()->q();
    seg_start_.reset();
    seg_dt_ = cfg_.period_ms / 1000.0;
    last_set_ = {};
    error_.clear();
    detail_.clear();

    nlohmann::json init = {{"lookahead", cfg_.lookahead_ms},
                           {"t", cfg_.period_ms},
                           {"smoothness", cfg_.smoothness},
                           {"response_enable", 0}};
    const Reply ok = co_await robot_.rpc().call("transparent_transmission_init", init);
    if (ok.ec) co_return ok.ec;
    if (ok.value == false) co_return make_error_code(StreamErrc::init_rejected);

    ticks_ = overruns_ = 0;
    stopping_ = false;
    running_ = true;
    asio::co_spawn(context_, run(), asio::detached);
    co_return std::error_code{};
}

asio::awaitable<std::error_code> Streamer::stop(bool settle) {
    std::error_code ignored;
    if (running_) {
        if (settle && !error_) {
            target_ = cmd_;          // stop moving the setpoint...
            seg_start_.reset();
            asio::steady_timer drain(context_);   // ...and let the controller's lookahead window empty
            drain.expires_after(std::chrono::milliseconds(cfg_.lookahead_ms + 200));
            co_await drain.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
        }
        stopping_ = true;
        tick_.cancel();              // the loop wakes and ends
        while (running_) {           // run() cancels loop_done_ on its way out
            loop_done_.expires_after(std::chrono::seconds(1));
            co_await loop_done_.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
        }
    }
    co_await robot_.rpc().call("tt_clear_servo_joint_buf");
    // don't return until the robot has left "running", so the next command sees fresh state
    co_await robot_.monitor().wait_until([](const RobotStatus& s) { return s.robot_state != State::Running; },
                                         std::chrono::milliseconds(2000));
    co_return error_;
}

// --- targets -----------------------------------------------------------------------------------
std::error_code Streamer::set_target(const Joints& q_in) {
    if (error_) return error_;
    if (!running_) return make_error_code(StreamErrc::not_running);
    const auto now = Clock::now();
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = std::clamp(q_in[i], cfg_.joint_min[i], cfg_.joint_max[i]);
    seg_start_ = interp(now);        // continue from where we are now
    seg_t0_ = now;
    const double gap = last_set_.time_since_epoch().count() == 0
                           ? cfg_.period_ms / 1000.0
                           : std::chrono::duration<double>(now - last_set_).count();
    seg_dt_ = std::clamp(gap, cfg_.period_ms / 1000.0, 0.5);
    target_ = q;
    last_set_ = now;
    return {};
}

std::error_code Streamer::set_target_delta(const Joints& dq) {
    Joints q = target_;
    for (int i = 0; i < 6; ++i) q[i] += dq[i];
    return set_target(q);
}

Joints Streamer::interp(Clock::time_point now) const {
    if (!seg_start_) return target_;
    const double a = std::min(1.0, std::chrono::duration<double>(now - seg_t0_).count() / seg_dt_);
    Joints q{};
    for (int i = 0; i < 6; ++i) q[i] = (*seg_start_)[i] + a * (target_[i] - (*seg_start_)[i]);
    return q;
}

bool Streamer::at_target(double tol) const {
    for (int i = 0; i < 6; ++i)
        if (std::abs(cmd_[i] - target_[i]) > tol) return false;
    return true;
}

// --- the loop ------------------------------------------------------------------------------------
std::error_code Streamer::abort_check(const Joints& cmd) {
    const auto s = robot_.status();
    if (!s || s->age_seconds() > 0.1) {
        detail_ = "monitor data older than 100 ms";
        return make_error_code(StreamErrc::stale_state);
    }
    if (s->robot_state == State::EStop || s->robot_state == State::Alarm || s->robot_state == State::Collision) {
        detail_ = std::string("robot entered state '") + to_string(s->robot_state) + "'";
        return make_error_code(StreamErrc::robot_fault);
    }
    if (s->robot_mode != Mode::Remote) {
        detail_ = "robot left remote mode";
        return make_error_code(StreamErrc::left_remote);
    }
    double err = 0;
    const Joints q = s->q();
    for (int i = 0; i < 6; ++i) err = std::max(err, std::abs(q[i] - cmd[i]));
    if (err > cfg_.max_tracking_error) {
        char text[100];
        std::snprintf(text, sizeof text, "tracking error %.3g deg exceeds the %.3g deg limit", err, cfg_.max_tracking_error);
        detail_ = text;
        return make_error_code(StreamErrc::tracking_error);
    }
    return {};
}

asio::awaitable<void> Streamer::run() {
    tick_.expires_after(std::chrono::milliseconds(0));
    while (!stopping_) {
        // setpoint: interpolate towards the target, then limit the speed
        const Joints goal = interp(Clock::now());
        for (int i = 0; i < 6; ++i) cmd_[i] += std::clamp(goal[i] - cmd_[i], -max_step_, max_step_);
        nlohmann::json put = {{"targetPos", cmd_}};
        robot_.rpc().notify("tt_put_servo_joint_to_buf", put);
        ++ticks_;

        if (std::error_code ec = abort_check(cmd_)) {
            error_ = ec;
            co_await robot_.stop();
            break;
        }

        // fixed-rate schedule without drift; after a stall, skip ahead instead of bursting
        tick_.expires_at(tick_.expiry() + period_);
        if (tick_.expiry() < asio::steady_timer::clock_type::now() - period_) {
            ++overruns_;
            tick_.expires_after(period_);
        }
        std::error_code ignored;
        co_await tick_.async_wait(asio::redirect_error(asio::use_awaitable, ignored));   // stop() cancels it
    }
    running_ = false;
    loop_done_.cancel();   // wakes stop()
}

}  // namespace ec
