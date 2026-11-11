#include "twin_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <future>

// GCC 11-13 coroutine pitfalls, avoided throughout: no braced lists inside a co_await
// expression, no structured bindings on a co_await result.

using namespace std::chrono_literals;

namespace twin {
namespace {

double max_abs_diff(const Joints& a, const Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

// Joints at time t on a piecewise-linear trajectory
Joints sample(const ec::planning::Trajectory& traj, double t) {
    if (t <= traj.times.front()) return traj.points.front();
    if (t >= traj.times.back()) return traj.points.back();
    const auto it = std::upper_bound(traj.times.begin(), traj.times.end(), t);
    const std::size_t i = static_cast<std::size_t>(it - traj.times.begin());
    const double s = (t - traj.times[i - 1]) / (traj.times[i] - traj.times[i - 1]);
    Joints q;
    for (int j = 0; j < 6; ++j) q[j] = traj.points[i - 1][j] + s * (traj.points[i][j] - traj.points[i - 1][j]);
    return q;
}

}  // namespace

const char* to_string(Source s) {
    switch (s) {
        case Source::Operator: return "operator";
        case Source::External: return "external";
        default: return "none";
    }
}

// --- lifetime --------------------------------------------------------------------------------
Core::Core(const mjModel* model, std::string ip, ec::planning::Options plan_options)
    : ip_(std::move(ip)),
      planner_(model, std::move(plan_options)),
      robot_work_(robot_context_.get_executor()),
      planning_work_(planning_context_.get_executor()),
      robot_(std::make_unique<ec::Robot>(robot_context_, ip_)),
      robot_thread_([this] { robot_context_.run(); }),
      planning_thread_([this] { planning_context_.run(); }) {
    limits_.max_velocity.fill(30);
    limits_.max_acceleration.fill(60);
    asio::co_spawn(robot_context_, watch_permissions(), asio::detached);
    connect_monitor();
}

Core::~Core() {
    // A motion still running: stop the arm before the connection goes away
    if (active_ != Source::None) {
        auto sent = std::make_shared<std::promise<void>>();
        stop_with("the twin closing", "stopped: the twin is closing", [sent](const Outcome&) { sent->set_value(); });
        sent->get_future().wait_for(1s);
    }
    asio::post(robot_context_, [this] { robot_->close(); });
    robot_work_.reset();
    planning_work_.reset();
    robot_context_.stop();
    planning_context_.stop();
    robot_thread_.join();
    planning_thread_.join();
    robot_.reset();   // sockets go before their io_context
}

// --- view (any thread) -------------------------------------------------------------------------
bool Core::can_arm() const {
    const auto s = robot_->status();
    return robot_->rpc().connected() && fresh(s) && s->robot_mode == ec::Mode::Remote;
}

View Core::view() const {
    View v;
    v.status = robot_->status();
    v.monitor_connected = robot_->monitor().connected();
    v.connecting = connecting_;
    v.packets = robot_->monitor().packets();
    v.invalid_packets = robot_->monitor().invalid_packets();
    v.rpc_connected = robot_->rpc().connected();
    v.armed = armed_;
    v.external = external_;
    v.can_arm = can_arm();
    v.active = active_;
    v.progress = progress_;
    v.busy = busy_ > 0;
    std::lock_guard<std::mutex> lock(mutex_);
    v.motion_text = motion_text_;
    v.last = last_;
    v.plan = plan_;
    return v;
}

void Core::finish(const Outcome& o, const Callback& done) {
    std::printf("%s\n", o.message.c_str());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_ = o;
    }
    if (done) done(o);
}

// --- connections ------------------------------------------------------------------------------------
void Core::connect_monitor() {
    asio::post(robot_context_, [this] {
        if (connecting_.exchange(true)) return;
        asio::co_spawn(robot_context_, [this]() -> asio::awaitable<void> {
            const std::error_code ec = co_await robot_->connect_monitor();
            connecting_ = false;
            finish({!ec, "monitor " + ip_ + ":8056: " + (ec ? robot_->last_error_detail() : "ok")}, {});
        }, asio::detached);
    });
}

void Core::connect_robot() {
    asio::post(robot_context_, [this] {
        if (connecting_.exchange(true)) return;
        asio::co_spawn(robot_context_, [this]() -> asio::awaitable<void> {
            const std::error_code ec = co_await robot_->connect_rpc();
            connecting_ = false;
            finish({!ec, "connect " + ip_ + ":8055: " + (ec ? robot_->last_error_detail() : "ok")}, {});
        }, asio::detached);
    });
}

void Core::disconnect_robot() {
    asio::post(robot_context_, [this] {
        armed_ = false;
        external_ = false;
        robot_->close_rpc();
        finish({true, "disconnect 8055: ok"}, {});
    });
}

// --- permissions (robot thread) ------------------------------------------------------------------------
void Core::set_armed(bool on) {
    asio::post(robot_context_, [this, on] {
        armed_ = on && can_arm();
        if (!armed_ && external_) {
            external_ = false;
            if (active_ == Source::External) cancel_motion("commands disarmed");
        }
    });
}

void Core::set_external_allowed(bool on) {
    asio::post(robot_context_, [this, on] {
        external_ = on && armed_;
        if (!external_ && active_ == Source::External) cancel_motion("the operator withdrew external control");
    });
}

asio::awaitable<void> Core::watch_permissions() {
    asio::steady_timer tick(robot_context_);
    tick.expires_after(0ms);
    for (;;) {
        tick.expires_at(tick.expiry() + 50ms);
        std::error_code ec;
        co_await tick.async_wait(asio::redirect_error(asio::use_awaitable, ec));
        if (ec) co_return;
        if (armed_ && !can_arm()) {
            armed_ = false;
            if (active_ != Source::None) cancel_motion("permissions lost (8055, stale monitor or not in remote mode)");
        }
        if (external_ && !armed_) {
            external_ = false;
            if (active_ == Source::External) cancel_motion("the operator withdrew external control");
        }
    }
}

bool Core::allowed(Source from, std::string* why) const {
    auto fail = [why](const char* text) {
        if (why) *why = text;
        return false;
    };
    if (!robot_->rpc().connected()) return fail("8055 is not connected (twin: Commands)");
    if (from == Source::Operator) {
        if (!armed_) return fail("commands are not armed");
        if (active_ == Source::External) return fail("an external motion is running");
    } else {
        if (!armed_ || !external_)
            return fail("the operator has not granted external control (twin: \"accept external commands\")");
        if (active_ == Source::Operator) return fail("the operator is running a motion");
    }
    return true;
}

// --- simple commands ------------------------------------------------------------------------------------
void Core::command(std::string what, Source from, bool needs_permission,
                   std::function<asio::awaitable<std::error_code>()> body, Callback done) {
    asio::post(robot_context_, [this, what = std::move(what), from, needs_permission, body = std::move(body),
                                done = std::move(done)]() mutable {
        std::string why;
        if (needs_permission && !allowed(from, &why)) {
            finish({false, what + ": refused, " + why}, done);
            return;
        }
        ++busy_;
        asio::co_spawn(robot_context_, [this, what, body = std::move(body), done]() -> asio::awaitable<void> {
            const std::error_code ec = co_await body();
            --busy_;
            finish({!ec, what + ": " + (ec ? robot_->last_error_detail() : std::string("ok"))}, done);
        }, asio::detached);
    });
}

void Core::stop(const std::string& who, Callback done) { stop_with(who, "stopped by " + who, std::move(done)); }

void Core::cancel_motion(const std::string& reason) { stop_with(reason, "stopped: " + reason, {}); }

void Core::stop_with(const std::string& label, const std::string& reason, Callback done) {
    asio::post(robot_context_, [this, label, reason, done = std::move(done)] {
        cancel_reason_ = reason;   // before the flag: the motion reads it when it sees the flag
        cancel_ = true;            // an upload in progress sees it at its next point
        if (!robot_->rpc().connected()) {
            finish({false, "stop: 8055 is not connected"}, done);
            return;
        }
        // STOP does not queue behind anything: its request overtakes a running upload
        asio::co_spawn(robot_context_, [this, label, done]() -> asio::awaitable<void> {
            const std::error_code ec = co_await robot_->stop();
            co_await robot_->stop_trajectory();   // no trajectory running: an error we don't need
            finish({!ec, "stop (" + label + "): " + (ec ? robot_->last_error_detail() : std::string("ok"))}, done);
        }, asio::detached);
    });
}

void Core::power(bool on, Source from, Callback done) {
    command(on ? "power on" : "power off", from, true, [this, on]() -> asio::awaitable<std::error_code> {
        int tool;
        double speed;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tool = power_tool_;
            speed = power_speed_;
        }
        if (!on) co_return co_await robot_->power_off();
        if (std::error_code ec = co_await robot_->set_tool(tool)) co_return ec;
        if (std::error_code ec = co_await robot_->power_on()) co_return ec;
        co_return co_await robot_->set_speed(speed);
    }, std::move(done));
}

void Core::clear_alarm(Source from, Callback done) {
    command("clear alarm", from, true, [this]() -> asio::awaitable<std::error_code> {
        co_return co_await robot_->clear_alarm();
    }, std::move(done));
}

void Core::set_speed(double percent, Callback done) {
    command("set speed", Source::Operator, true, [this, percent]() -> asio::awaitable<std::error_code> {
        co_return co_await robot_->set_speed(percent);
    }, std::move(done));
}

void Core::move_joint(const Joints& q, double speed, Callback done) {
    command("moveByJoint", Source::Operator, true, [this, q, speed]() -> asio::awaitable<std::error_code> {
        if (active_ != Source::None) co_return make_error_code(ec::RobotErrc::not_stopped);
        co_return co_await robot_->move_joint(q, speed, /*wait=*/false);
    }, std::move(done));
}

void Core::set_limits(double speed, double acc) {
    std::lock_guard<std::mutex> lock(mutex_);
    limits_.max_velocity.fill(speed);
    limits_.max_acceleration.fill(acc);
}

void Core::set_power_settings(int tool, double speed_override) {
    std::lock_guard<std::mutex> lock(mutex_);
    power_tool_ = tool;
    power_speed_ = speed_override;
}

// --- planning thread ----------------------------------------------------------------------------------------
template <typename T>
asio::awaitable<T> Core::on_planning_thread(std::function<T()> fn) {
    // runs fn on the planning thread; the caller resumes on its own (robot) thread
    co_return co_await asio::co_spawn(planning_context_,
                                      [fn = std::move(fn)]() -> asio::awaitable<T> { co_return fn(); },
                                      asio::use_awaitable);
}

std::shared_ptr<Plan> Core::describe(std::string source, ec::planning::Trajectory traj) const {
    double closest = 1e9;
    for (std::size_t i = 0; i < traj.points.size(); i += 5) closest = std::min(closest, planner_.clearance(traj.points[i]));
    auto plan = std::make_shared<Plan>();
    char text[160];
    std::snprintf(text, sizeof text, "%zu points, %.1f s, closest approach %.0f mm", traj.points.size(),
                  traj.duration(), closest * 1000);
    plan->source = std::move(source);
    plan->summary = text;
    plan->traj = std::move(traj);
    return plan;
}

void Core::plan_to(const Joints& goal) {
    const auto s = robot_->status();
    if (!fresh(s)) {
        finish({false, "plan: no fresh robot state"}, {});
        return;
    }
    const Joints start = s->q();
    ++busy_;
    asio::post(planning_context_, [this, start, goal] {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            plan_.reset();
        }
        Outcome o;
        const auto result = planner_.plan(start, goal);
        if (!result.solved) {
            o = {false, "plan: " + result.message};
        } else {
            ec::planning::Limits lim;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                lim = limits_;
            }
            auto traj = planner_.time_parameterize(result.waypoints, lim);
            if (auto why = planner_.verify(traj)) {
                o = {false, "plan: not collision-free: " + *why};
            } else {
                auto plan = describe(result.attempts == 0 ? std::string("planned, straight line is free")
                                                          : "planned, " + std::to_string(result.waypoints.size()) + " waypoints",
                                     std::move(traj));
                o = {true, "plan: " + plan->summary};
                std::lock_guard<std::mutex> lock(mutex_);
                plan_ = std::move(plan);
            }
        }
        --busy_;
        finish(o, {});
    });
}

void Core::discard_plan() {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_.reset();
}

// --- motions --------------------------------------------------------------------------------------------------
void Core::start_motion(std::function<std::shared_ptr<const Plan>()> build, Source from, MotionCallbacks cb) {
    asio::post(robot_context_, [this, build = std::move(build), from, cb = std::move(cb)]() mutable {
        auto refuse = [&](const std::string& why) {
            finish({false, std::string("motion (") + to_string(from) + "): refused, " + why}, cb.done);
        };
        std::string why;
        if (!allowed(from, &why)) return refuse(why);
        if (active_ != Source::None) return refuse("a motion is already running");
        const auto s = robot_->status();
        if (!fresh(s)) return refuse("no fresh robot state");
        if (s->robot_state != ec::State::Stop)
            return refuse(std::string("the robot is not stopped (state: ") + ec::to_string(s->robot_state) + ")");
        cancel_ = false;
        cancel_reason_.clear();
        progress_ = 0;
        active_ = from;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_text_ = std::string(to_string(from)) + ": checking";
        }
        asio::co_spawn(robot_context_, motion(std::move(build), from, std::move(cb)), asio::detached);
    });
}

asio::awaitable<void> Core::motion(std::function<std::shared_ptr<const Plan>()> build, Source from, MotionCallbacks cb) {
    const std::string what = std::string("motion (") + to_string(from) + ")";
    // time + check on the planning thread; the robot thread stays free meanwhile (STOP, monitor)
    const std::shared_ptr<const Plan> plan = co_await on_planning_thread<std::shared_ptr<const Plan>>(std::move(build));
    Outcome o;
    if (!plan || plan->traj.points.empty()) {
        o = {false, what + ": refused, " + (plan ? plan->summary : std::string("no trajectory"))};
    } else if (cancel_) {
        o = {false, what + ": " + cancel_reason_};
    } else {
        o = co_await execute(*plan, from, cb);
        o.message = what + ": " + o.message;
    }
    active_ = Source::None;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_text_ = o.message;
    }
    finish(o, cb.done);
}

asio::awaitable<Outcome> Core::execute(const Plan& plan, Source from, const MotionCallbacks& cb) {
    const Joints& start = plan.traj.points.front();
    const Joints& goal = plan.traj.points.back();
    const auto s0 = robot_->status();
    const double off = s0 ? max_abs_diff(s0->q(), start) : 1e9;
    char text[160];
    if (off > 0.5) {
        std::snprintf(text, sizeof text, "the arm is %.2f deg from the motion's start (max 0.5): plan again", off);
        co_return Outcome{false, text};
    }
    // start == goal: nothing to move (the controller would refuse a trajectory this short anyway)
    if (plan.traj.points.size() < 3 || max_abs_diff(start, goal) < 0.01)
        co_return Outcome{true, "already at the goal, nothing to do"};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_text_ = std::string(to_string(from)) + ": uploading " + std::to_string(plan.traj.points.size()) + " points";
    }
    const std::error_code ec = co_await robot_->run_joint_trajectory(plan.traj.times, plan.traj.points, 100.0, 10.0,
                                                                    /*wait=*/false, std::chrono::seconds(120), &cancel_);
    if (ec) co_return Outcome{false, cancel_ ? cancel_reason_ : robot_->last_error_detail()};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_text_ = std::string(to_string(from)) + ": " + plan.source + ", " + plan.summary;
    }

    // follow it: progress, feedback, the end
    const double duration = plan.traj.duration();
    const auto uploaded = Clock::now();
    Clock::time_point t0, last_feedback;
    bool started = false;
    asio::steady_timer tick(robot_context_);
    for (;;) {
        tick.expires_after(10ms);
        std::error_code ignored;
        co_await tick.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
        if (cancel_) co_return Outcome{false, cancel_reason_, true};
        const auto s = robot_->status();
        if (!s || s->age_seconds() > 0.1) {
            co_await robot_->stop();
            co_return Outcome{false, "lost the robot state, stop sent", true};
        }
        const auto now = Clock::now();
        if (s->robot_state == ec::State::Running) {
            if (!started) {
                started = true;
                t0 = now;
            }
            const double t = std::chrono::duration<double>(now - t0).count();
            progress_ = std::min(1.0, t / duration);
            if (cb.feedback && now - last_feedback >= 50ms) {
                cb.feedback(t, sample(plan.traj, t), s->q());
                last_feedback = now;
            }
            if (t > duration + 10.0) {
                co_await robot_->stop();
                co_return Outcome{false, "the motion did not finish in time; stop sent", true};
            }
        } else if (started || now - uploaded > 3s) {
            if (s->robot_state != ec::State::Stop)
                co_return Outcome{false, std::string("the motion ended in state '") + ec::to_string(s->robot_state) + "'", true};
            const double error = max_abs_diff(s->q(), goal);
            if (error > 0.1) {
                std::snprintf(text, sizeof text, "the arm stopped %.2f deg from the goal", error);
                co_return Outcome{false, text, true};
            }
            progress_ = 1.0;
            co_return Outcome{true, "ok", true};
        }
    }
}

void Core::run_plan(std::shared_ptr<const Plan> plan, Source from, MotionCallbacks cb) {
    start_motion([plan] { return plan; }, from, std::move(cb));
}

// The build steps below run on the planning thread. A failed build returns a Plan with no
// trajectory and the reason in its summary (no exceptions).
namespace {
std::shared_ptr<const Plan> refused_plan(std::string why) {
    auto p = std::make_shared<Plan>();
    p->summary = std::move(why);
    return p;
}
}  // namespace

void Core::run_path(std::vector<Joints> waypoints, Source from, MotionCallbacks cb) {
    start_motion(
        [this, waypoints = std::move(waypoints)]() -> std::shared_ptr<const Plan> {
            if (waypoints.size() < 2) return refused_plan("a path needs at least 2 waypoints (start and goal)");
            ec::planning::Limits lim;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                lim = limits_;
            }
            auto traj = planner_.time_parameterize(waypoints, lim);   // the twin's limits
            if (auto why = planner_.verify(traj)) return refused_plan("not collision-free: " + *why);
            return describe("path of " + std::to_string(waypoints.size()) + " waypoints, timed by the twin", std::move(traj));
        },
        from, std::move(cb));
}

void Core::run_timed(ec::planning::Trajectory traj, Source from, MotionCallbacks cb) {
    start_motion(
        [this, traj = std::move(traj)]() -> std::shared_ptr<const Plan> {
            const auto n = traj.points.size();
            if (n < 2 || traj.times.size() != n) return refused_plan("need >= 2 points, one time each");
            if (traj.times.front() != 0.0) return refused_plan("times must start at 0");
            for (std::size_t i = 1; i < n; ++i)
                if (traj.times[i] <= traj.times[i - 1]) return refused_plan("times must increase");

            // Densify to 8 ms with straight lines in joint space: exactly what gets checked and sent.
            ec::planning::Trajectory dense;
            const double T = traj.times.back();
            const int steps = std::max(2, static_cast<int>(std::ceil(T / 0.008)));
            for (int k = 0; k <= steps; ++k) {
                const double t = T * k / steps;
                dense.times.push_back(t);
                dense.points.push_back(sample(traj, t));
            }

            // Speeds and accelerations of what will actually be sent, starting and ending at rest.
            // Sparse timed points become velocity jumps here, and are refused.
            ec::planning::Limits lim;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                lim = limits_;
            }
            const double dt = T / steps;
            char text[220];
            for (int k = 0; k <= steps + 1; ++k) {
                for (int j = 0; j < 6; ++j) {
                    auto vel = [&](int i) {   // velocity of the segment ending at sample i (0 outside)
                        return i < 1 || i > steps ? 0.0 : (dense.points[i][j] - dense.points[i - 1][j]) / dt;
                    };
                    const double v = vel(k), a = (vel(k) - vel(k - 1)) / dt;
                    const double t = std::min(k, steps) * dt;
                    if (std::abs(v) > lim.max_velocity[j] * 1.05) {
                        std::snprintf(text, sizeof text, "joint %d moves at %.1f deg/s at t = %.3f s; the twin's limit is %.0f",
                                      j + 1, std::abs(v), t, lim.max_velocity[j]);
                        return refused_plan(text);
                    }
                    if (std::abs(a) > lim.max_acceleration[j] * 1.25) {
                        std::snprintf(text, sizeof text,
                                      "joint %d accelerates at %.0f deg/s^2 at t = %.3f s; the twin's limit is %.0f "
                                      "(a velocity jump between sparse points counts too)",
                                      j + 1, std::abs(a), t, lim.max_acceleration[j]);
                        return refused_plan(text);
                    }
                }
            }
            if (auto why = planner_.verify(dense)) return refused_plan("not collision-free: " + *why);
            return describe("timed trajectory of " + std::to_string(n) + " points (sender's timing)", std::move(dense));
        },
        from, std::move(cb));
}

}  // namespace twin
