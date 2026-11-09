#include "twin_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

using namespace std::chrono_literals;

namespace twin {
namespace {

double max_abs_diff(const Joints& a, const Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

std::future<Outcome> ready(bool ok, std::string message) {
    std::promise<Outcome> p;
    p.set_value({ok, std::move(message)});
    return p.get_future();
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

// --- Worker ------------------------------------------------------------------------------
Worker::Worker() : thread_([this] { io_.run(); }) {}

Worker::~Worker() {
    guard_.reset();
    io_.stop();
    thread_.join();
}

bool Worker::run(std::string what, std::function<void()> fn, std::function<void(const Outcome&)> done) {
    if (busy_.exchange(true)) return false;
    asio::post(io_, [this, what = std::move(what), fn = std::move(fn), done = std::move(done)] {
        Outcome outcome;
        try {
            fn();
            outcome = {true, what + ": ok"};
        } catch (const std::exception& e) {
            outcome = {false, what + ": " + e.what()};
        }
        std::printf("%s\n", outcome.message.c_str());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_ = outcome;
        }
        busy_ = false;   // before done(): a callback may want to start the next job
        if (done) done(outcome);
    });
    return true;
}

Outcome Worker::last() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_;
}

// --- Core: connections ---------------------------------------------------------------------
Core::Core(const mjModel* model, std::string ip, ec::planning::Options plan_options)
    : ip_(std::move(ip)), planner_(model, std::move(plan_options)) {
    limits_.max_velocity.fill(30);
    limits_.max_acceleration.fill(60);
    connect_monitor();
}

Core::~Core() {
    cancel_ = true;   // let a running motion loop end
}

void Core::connect_monitor() {
    jobs_.run("monitor " + ip_ + ":8056", [this] {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            monitor_.reset();
        }
        auto m = std::make_shared<ec::Monitor>(ip_);
        m->start(2000ms);
        std::lock_guard<std::mutex> lock(mutex_);
        monitor_ = std::move(m);
    });
}

std::shared_ptr<ec::Monitor> Core::monitor() {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitor_;
}

std::optional<ec::RobotStatus> Core::status() {
    auto m = monitor();
    if (!m) return std::nullopt;
    try {
        return m->latest(0ms);   // 0 = no age check: the caller decides
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::shared_ptr<ec::Robot> Core::robot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return robot_;
}

void Core::connect_robot() {
    jobs_.run("connect " + ip_ + ":8055", [this] {
        auto r = std::make_shared<ec::Robot>(ip_);
        r->connect();
        // A controller that already has a client accepts the connection and closes it at once
        try {
            r->version();
        } catch (const std::exception&) {
            throw std::runtime_error("8055 closed the connection: another client holds it");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        robot_ = std::move(r);
    });
}

void Core::disconnect_robot() {
    set_armed(false);
    jobs_.run("disconnect 8055", [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_.reset();
    });
}

// --- permissions -----------------------------------------------------------------------------
bool Core::can_arm() {
    const auto s = status();
    return robot() && fresh(s) && s->robot_mode == ec::Mode::Remote;
}

void Core::set_armed(bool on) {
    armed_ = on && can_arm();
    if (!armed_) set_external_allowed(false);
}

void Core::set_external_allowed(bool on) {
    external_ = on && armed_;
    if (!external_ && active_ == Source::External) cancel_motion("the operator withdrew external control");
}

void Core::update_permissions() {
    if (armed_ && !can_arm()) {
        armed_ = false;
        if (active_ != Source::None) cancel_motion("permissions lost (8055, stale monitor or not in remote mode)");
    }
    if (external_ && !armed_) set_external_allowed(false);
}

bool Core::allowed(Source from, std::string* why) {
    auto fail = [why](const char* text) {
        if (why) *why = text;
        return false;
    };
    if (!robot()) return fail("8055 is not connected (twin: Commands)");
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

std::string Core::motion_text() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return motion_text_;
}

ec::planning::Limits Core::limits() {
    std::lock_guard<std::mutex> lock(mutex_);
    return limits_;
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

// --- simple commands ----------------------------------------------------------------------------
std::future<Outcome> Core::submit(const std::string& what, Source from, bool needs_permission,
                                  std::function<void()> fn) {
    std::string why;
    if (needs_permission && !allowed(from, &why)) return ready(false, what + ": refused, " + why);
    auto promise = std::make_shared<std::promise<Outcome>>();
    auto future = promise->get_future();
    if (!jobs_.run(what, std::move(fn), [promise](const Outcome& o) { promise->set_value(o); }))
        return ready(false, what + ": refused, the twin is busy");
    return future;
}

std::future<Outcome> Core::stop(const std::string& who) { return stop_with(who, "stopped by " + who); }

void Core::cancel_motion(const std::string& reason) { stop_with(reason, "stopped: " + reason); }

std::future<Outcome> Core::stop_with(const std::string& label, const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cancel_reason_ = reason;   // before the flag: the motion loop reads it when it sees the flag
    }
    cancel_ = true;   // at once, on the caller's thread: an upload in progress sees it at the next point
    auto r = robot();
    if (!r) return ready(false, "stop: 8055 is not connected");
    auto promise = std::make_shared<std::promise<Outcome>>();
    auto future = promise->get_future();
    const bool started = urgent_.run(
        "stop (" + label + ")",
        [r] {
            r->stop();
            try { r->stop_trajectory(); } catch (const std::exception&) {}   // no trajectory: fine
        },
        [promise](const Outcome& o) { promise->set_value(o); });
    if (!started) return ready(true, "stop: already being sent");
    return future;
}

std::future<Outcome> Core::power(bool on, Source from) {
    int tool;
    double speed;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tool = power_tool_;
        speed = power_speed_;
    }
    return submit(on ? "power on" : "power off", from, true, [this, on, tool, speed] {
        auto r = robot();
        if (!r) throw std::runtime_error("8055 is not connected");
        if (on) {
            r->set_tool(tool);
            r->power_on();
            r->set_speed(speed);
        } else {
            r->power_off();
        }
    });
}

std::future<Outcome> Core::clear_alarm(Source from) {
    return submit("clear alarm", from, true, [this] {
        auto r = robot();
        if (!r) throw std::runtime_error("8055 is not connected");
        r->rpc().call("clearAlarm");
    });
}

std::future<Outcome> Core::set_speed(double percent) {
    return submit("set speed", Source::Operator, true, [this, percent] { robot()->set_speed(percent); });
}

std::future<Outcome> Core::move_joint(const Joints& q, double speed) {
    if (active_ != Source::None) return ready(false, "moveByJoint: refused, a motion is running");
    return submit("moveByJoint", Source::Operator, true, [this, q, speed] { robot()->move_joint(q, speed, false); });
}

// --- motions ----------------------------------------------------------------------------------------
std::shared_ptr<Plan> Core::describe(std::string source, ec::planning::Trajectory traj) const {
    if (auto why = planner_.verify(traj)) throw std::runtime_error("not collision-free: " + *why);
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

bool Core::start_motion(std::function<std::shared_ptr<const Plan>()> build, Source from, MotionCallbacks cb,
                        std::string* why) {
    if (!allowed(from, why)) return false;
    if (active_ != Source::None) {
        if (why) *why = "a motion is already running";
        return false;
    }
    const auto s = status();
    if (!fresh(s)) {
        if (why) *why = "no fresh robot state";
        return false;
    }
    if (s->robot_state != ec::State::Stop) {
        if (why) *why = std::string("the robot is not stopped (state: ") + ec::to_string(s->robot_state) + ")";
        return false;
    }
    cancel_ = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cancel_reason_.clear();
        motion_text_ = std::string(to_string(from)) + ": checking";
    }
    progress_ = 0;
    active_ = from;
    const bool started = jobs_.run(
        std::string("motion (") + to_string(from) + ")",
        [this, build = std::move(build), from, cb] {
            const std::shared_ptr<const Plan> plan = build();
            execute(*plan, from, cb);
        },
        [this, cb](const Outcome& o) {
            active_ = Source::None;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                motion_text_ = o.message;
            }
            if (cb.done) cb.done(o);
        });
    if (!started) {
        active_ = Source::None;
        if (why) *why = "the twin is busy";
        return false;
    }
    return true;
}

bool Core::run_plan(std::shared_ptr<const Plan> plan, Source from, MotionCallbacks cb, std::string* why) {
    return start_motion([plan] { return plan; }, from, std::move(cb), why);
}

bool Core::run_path(std::vector<Joints> waypoints, Source from, MotionCallbacks cb, std::string* why) {
    if (waypoints.size() < 2) {
        if (why) *why = "a path needs at least 2 waypoints (start and goal)";
        return false;
    }
    return start_motion(
        [this, waypoints = std::move(waypoints)]() -> std::shared_ptr<const Plan> {
            auto traj = planner_.time_parameterize(waypoints, limits());   // the twin's limits
            return describe("path of " + std::to_string(waypoints.size()) + " waypoints, timed by the twin",
                            std::move(traj));
        },
        from, std::move(cb), why);
}

bool Core::run_timed(ec::planning::Trajectory traj, Source from, MotionCallbacks cb, std::string* why) {
    return start_motion(
        [this, traj = std::move(traj)]() -> std::shared_ptr<const Plan> {
            const auto n = traj.points.size();
            if (n < 2 || traj.times.size() != n) throw std::runtime_error("need >= 2 points, one time each");
            if (traj.times.front() != 0.0) throw std::runtime_error("times must start at 0");
            for (std::size_t i = 1; i < n; ++i)
                if (traj.times[i] <= traj.times[i - 1]) throw std::runtime_error("times must increase");

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
            // Sparse timed points become velocity steps here, and are refused.
            const ec::planning::Limits lim = limits();
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
                        throw std::runtime_error(text);
                    }
                    if (std::abs(a) > lim.max_acceleration[j] * 1.25) {
                        std::snprintf(text, sizeof text,
                                      "joint %d accelerates at %.0f deg/s^2 at t = %.3f s; the twin's limit is %.0f "
                                      "(a velocity jump between sparse points counts too)",
                                      j + 1, std::abs(a), t, lim.max_acceleration[j]);
                        throw std::runtime_error(text);
                    }
                }
            }
            return describe("timed trajectory of " + std::to_string(n) + " points (sender's timing)", std::move(dense));
        },
        from, std::move(cb), why);
}

void Core::execute(const Plan& plan, Source from, const MotionCallbacks& cb) {
    auto r = robot();
    if (!r) throw std::runtime_error("8055 is not connected");
    auto cancelled_reason = [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_reason_.empty() ? std::string("stopped") : cancel_reason_;
    };

    const Joints& start = plan.traj.points.front();
    const Joints& goal = plan.traj.points.back();
    const double off = max_abs_diff(r->status().q(), start);
    if (off > 0.5) {
        char text[120];
        std::snprintf(text, sizeof text, "the arm is %.2f deg from the motion's start (max 0.5): plan again", off);
        throw std::runtime_error(text);
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_text_ = std::string(to_string(from)) + ": uploading " + std::to_string(plan.traj.points.size()) + " points";
    }
    try {
        r->run_joint_trajectory(plan.traj.times, plan.traj.points, 100.0, 10.0, false, std::chrono::seconds(120),
                                &cancel_);
    } catch (const std::exception&) {
        if (cancel_) throw std::runtime_error(cancelled_reason());
        throw;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_text_ = std::string(to_string(from)) + ": " + plan.source + ", " + plan.summary;
    }

    const double duration = plan.traj.duration();
    const auto uploaded = Clock::now();
    Clock::time_point t0, last_feedback;
    bool started = false;
    while (true) {
        std::this_thread::sleep_for(10ms);
        if (cancel_) throw std::runtime_error(cancelled_reason());
        ec::RobotStatus s;
        try {
            s = r->status();   // throws when the monitor is stale (> 100 ms)
        } catch (const std::exception& e) {
            r->stop();
            throw std::runtime_error(std::string("lost the robot state, stop sent: ") + e.what());
        }
        const auto now = Clock::now();
        if (s.robot_state == ec::State::Running) {
            if (!started) {
                started = true;
                t0 = now;
            }
            const double t = std::chrono::duration<double>(now - t0).count();
            progress_ = std::min(1.0, t / duration);
            if (cb.feedback && now - last_feedback >= 50ms) {
                cb.feedback(t, sample(plan.traj, t), s.q());
                last_feedback = now;
            }
            if (t > duration + 10.0) {
                r->stop();
                throw std::runtime_error("the motion did not finish in time; stop sent");
            }
        } else if (started || now - uploaded > 3s) {
            if (s.robot_state != ec::State::Stop)
                throw std::runtime_error(std::string("the motion ended in state '") + ec::to_string(s.robot_state) + "'");
            const double error = max_abs_diff(s.q(), goal);
            if (error > 0.1) {
                char text[100];
                std::snprintf(text, sizeof text, "the arm stopped %.2f deg from the goal", error);
                throw std::runtime_error(text);
            }
            progress_ = 1.0;
            return;
        }
    }
}

// --- operator planning ---------------------------------------------------------------------------------
std::future<Outcome> Core::plan_to(const Joints& goal) {
    const auto s = status();
    if (!fresh(s)) return ready(false, "plan: no fresh robot state");
    const Joints start = s->q();
    return submit("plan", Source::Operator, false, [this, start, goal] {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            plan_.reset();
        }
        const auto result = planner_.plan(start, goal);
        if (!result.solved) throw std::runtime_error(result.message);
        auto traj = planner_.time_parameterize(result.waypoints, limits());
        auto plan = describe(result.attempts == 0 ? std::string("planned, straight line is free")
                                                  : "planned, " + std::to_string(result.waypoints.size()) + " waypoints",
                             std::move(traj));
        std::lock_guard<std::mutex> lock(mutex_);
        plan_ = std::move(plan);
    });
}

std::shared_ptr<const Plan> Core::plan() {
    std::lock_guard<std::mutex> lock(mutex_);
    return plan_;
}

void Core::discard_plan() {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_.reset();
}

}  // namespace twin
