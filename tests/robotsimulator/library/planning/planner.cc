#include "planning/planner.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ec::planning {
namespace {

constexpr double kDeg = mjPI / 180.0;
constexpr double kReportRange = 0.05;   // m beyond the margin in which clearance is still measured

double max_abs_diff(const Joints& a, const Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

std::string geom_label(const mjModel* m, int geom) {
    if (const char* name = mj_id2name(m, mjOBJ_GEOM, geom); name && *name) return name;
    const int body = m->geom_bodyid[geom];
    if (const char* name = mj_id2name(m, mjOBJ_BODY, body); name && *name) return name;
    return body == 0 ? "world" : "body " + std::to_string(body);
}

ob::PlannerPtr make_planner(const std::string& name, const ob::SpaceInformationPtr& si) {
    if (name == "RRTConnect") return std::make_shared<og::RRTConnect>(si);
    if (name == "RRT") return std::make_shared<og::RRT>(si);
    if (name == "RRTstar") return std::make_shared<og::RRTstar>(si);   // optimises: uses the whole timeout
    if (name == "PRM") return std::make_shared<og::PRM>(si);
    if (name == "KPIECE1") return std::make_shared<og::KPIECE1>(si);
    if (name == "BKPIECE1") return std::make_shared<og::BKPIECE1>(si);
    throw std::invalid_argument("unknown planner '" + name +
                                "' (RRTConnect, RRT, RRTstar, PRM, KPIECE1, BKPIECE1)");
}

}  // namespace

Planner::Planner(const mjModel* model, Options options) : options_(std::move(options)) {
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    make_planner(options_.algorithm, std::make_shared<ob::SpaceInformation>(
                                         std::make_shared<ob::RealVectorStateSpace>(1)));   // validate the name now

    // Private copy: we change the contact margins, and the simulation must not see that.
    m_ = mj_copyModel(nullptr, model);
    d_ = mj_makeData(m_);
    for (int g = 0; g < m_->ngeom; ++g) {
        m_->geom_margin[g] = options_.margin + kReportRange;   // contacts are generated within this distance
        m_->geom_gap[g] = 0;
    }

    // The arm's joints are the ones the first six actuators drive (scene.xml: act1..act6)
    if (m_->nu < 6) throw std::runtime_error("planner: the model needs the 6 arm actuators");
    int joint1_body = -1;
    for (int i = 0; i < 6; ++i) {
        if (m_->actuator_trntype[i] != mjTRN_JOINT) throw std::runtime_error("planner: actuator is not on a joint");
        const int j = m_->actuator_trnid[2 * i];
        qpos_adr_[i] = m_->jnt_qposadr[j];
        if (m_->jnt_limited[j]) {
            lower_[i] = m_->jnt_range[2 * j] / kDeg;
            upper_[i] = m_->jnt_range[2 * j + 1] / kDeg;
        } else {
            lower_[i] = -360;
            upper_[i] = 360;
        }
        if (i == 0) joint1_body = m_->jnt_bodyid[j];
    }

    // Moving arm bodies: joint 1's body and everything below it. Arm: the whole tree
    // joint 1 hangs from (base_link included).
    moving_arm_.assign(m_->nbody, 0);
    arm_.assign(m_->nbody, 0);
    for (int b = 1; b < m_->nbody; ++b) {
        for (int a = b; a > 0; a = m_->body_parentid[a]) {
            if (a == joint1_body) { moving_arm_[b] = 1; break; }
        }
        arm_[b] = m_->body_rootid[b] == m_->body_rootid[joint1_body];
    }
    mj_forward(m_, d_);   // positions of everything else at the model's defaults
    find_relaxed_pairs();
}

void Planner::find_relaxed_pairs() {
    // Arm-arm geom pairs within the margin in every one of N random poses
    std::mt19937 rng(12345);
    std::set<std::pair<int, int>> always;
    constexpr int kSamples = 500;
    for (int n = 0; n < kSamples; ++n) {
        Joints q;
        for (int i = 0; i < 6; ++i) q[i] = std::uniform_real_distribution<double>(lower_[i], upper_[i])(rng);
        set_arm(q);
        std::set<std::pair<int, int>> close;
        for (int i = 0; i < d_->ncon; ++i) {
            const mjContact& c = d_->contact[i];
            if (c.geom[0] < 0 || c.geom[1] < 0 || c.dist >= options_.margin) continue;
            if (!arm_[m_->geom_bodyid[c.geom[0]]] || !arm_[m_->geom_bodyid[c.geom[1]]]) continue;
            close.insert(std::pair<int, int>(std::minmax(c.geom[0], c.geom[1])));
        }
        if (n == 0) {
            always = close;
        } else {
            std::set<std::pair<int, int>> both;
            for (const auto& p : always) if (close.count(p)) both.insert(p);
            always.swap(both);
        }
        if (always.empty()) break;
    }
    relaxed_.assign(always.begin(), always.end());
}

bool Planner::relaxed(int g1, int g2) const {
    const std::pair<int, int> key = std::minmax(g1, g2);
    return std::find(relaxed_.begin(), relaxed_.end(), key) != relaxed_.end();
}

std::vector<std::string> Planner::relaxed_pairs() const {
    std::vector<std::string> out;
    for (const auto& [a, b] : relaxed_) out.push_back(geom_label(m_, a) + " / " + geom_label(m_, b));
    return out;
}

Planner::~Planner() {
    mj_deleteData(d_);
    mj_deleteModel(m_);
}

void Planner::update_scene(const mjData* d) {
    for (int j = 0; j < m_->njnt; ++j) {
        if (moving_arm_[m_->jnt_bodyid[j]]) continue;
        const int adr = m_->jnt_qposadr[j];
        const int n = m_->jnt_type[j] == mjJNT_FREE ? 7 : m_->jnt_type[j] == mjJNT_BALL ? 4 : 1;
        for (int k = 0; k < n; ++k) d_->qpos[adr + k] = d->qpos[adr + k];
    }
    for (int i = 0; i < 3 * m_->nmocap; ++i) d_->mocap_pos[i] = d->mocap_pos[i];
    for (int i = 0; i < 4 * m_->nmocap; ++i) d_->mocap_quat[i] = d->mocap_quat[i];
}

void Planner::set_arm(const Joints& q) const {
    for (int i = 0; i < 6; ++i) d_->qpos[qpos_adr_[i]] = q[i] * kDeg;
    mj_kinematics(m_, d_);
    mj_collision(m_, d_);
}

Planner::Worst Planner::worst_contact(const Joints& q) const {
    set_arm(q);
    Worst worst{kReportRange, options_.margin + kReportRange, -1, -1};
    for (int i = 0; i < d_->ncon; ++i) {
        const mjContact& c = d_->contact[i];
        if (c.geom[0] < 0 || c.geom[1] < 0) continue;   // flex contacts: not used here
        if (!moving_arm_[m_->geom_bodyid[c.geom[0]]] && !moving_arm_[m_->geom_bodyid[c.geom[1]]]) continue;
        const double required = relaxed(c.geom[0], c.geom[1]) ? 0.0 : options_.margin;
        if (c.dist - required < worst.slack) worst = {c.dist - required, c.dist, c.geom[0], c.geom[1]};
    }
    return worst;
}

double Planner::clearance(const Joints& q) const {
    set_arm(q);
    double best = options_.margin + kReportRange;
    for (int i = 0; i < d_->ncon; ++i) {
        const mjContact& c = d_->contact[i];
        if (c.geom[0] < 0 || c.geom[1] < 0 || relaxed(c.geom[0], c.geom[1])) continue;
        if (!moving_arm_[m_->geom_bodyid[c.geom[0]]] && !moving_arm_[m_->geom_bodyid[c.geom[1]]]) continue;
        best = std::min(best, c.dist);
    }
    return best;
}

std::optional<std::string> Planner::check(const Joints& q) const {
    char text[200];
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(q[i]) || q[i] < lower_[i] || q[i] > upper_[i]) {
            std::snprintf(text, sizeof text, "joint %d at %.2f deg is outside [%.1f, %.1f]", i + 1, q[i],
                          lower_[i], upper_[i]);
            return std::string(text);
        }
    }
    const Worst w = worst_contact(q);
    if (w.slack >= 0) return std::nullopt;
    std::snprintf(text, sizeof text, "%s / %s: %.1f mm (needs %.0f mm)", geom_label(m_, w.geom1).c_str(),
                  geom_label(m_, w.geom2).c_str(), w.distance * 1000, (w.distance - w.slack) * 1000);
    return std::string(text);
}

Result Planner::plan(const Joints& start, const Joints& goal) const {
    Result result;
    if (auto why = check(start)) { result.message = "start is invalid: " + *why; return result; }
    if (auto why = check(goal)) { result.message = "goal is invalid: " + *why; return result; }

    auto space = std::make_shared<ob::RealVectorStateSpace>(6);   // radians
    ob::RealVectorBounds bounds(6);
    for (int i = 0; i < 6; ++i) {
        bounds.setLow(i, lower_[i] * kDeg);
        bounds.setHigh(i, upper_[i] * kDeg);
    }
    space->setBounds(bounds);

    og::SimpleSetup setup(space);
    const ob::SpaceInformationPtr si = setup.getSpaceInformation();
    setup.setStateValidityChecker([this, si](const ob::State* state) {
        if (!si->satisfiesBounds(state)) return false;
        const double* v = state->as<ob::RealVectorStateSpace::StateType>()->values;
        Joints q;
        for (int i = 0; i < 6; ++i) q[i] = v[i] / kDeg;
        return worst_contact(q).slack >= 0;
    });
    // Motions are checked every check_step_deg (as a fraction of the space's largest extent)
    si->setStateValidityCheckingResolution(options_.check_step_deg * kDeg / space->getMaximumExtent());

    ob::ScopedState<> s(space), g(space);
    for (int i = 0; i < 6; ++i) {
        s[i] = start[i] * kDeg;
        g[i] = goal[i] * kDeg;
    }
    setup.setStartAndGoalStates(s, g);
    setup.setPlanner(make_planner(options_.algorithm, si));
    setup.setup();

    auto travel = [](const std::vector<Joints>& path) {
        double sum = 0;
        for (std::size_t i = 1; i < path.size(); ++i) sum += max_abs_diff(path[i - 1], path[i]);
        return sum;
    };

    // The straight move is the best possible path: if it is free, no search is needed
    if (si->checkMotion(s.get(), g.get())) {
        result.waypoints = {start, goal};
    } else {
        const auto t0 = std::chrono::steady_clock::now();
        auto elapsed = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
        const int attempts = std::max(1, options_.attempts);
        std::string last_status;
        for (int a = 0; a < attempts; ++a) {
            // a fair share of what is left, so early failures leave time for later attempts
            const double budget = (options_.timeout - elapsed()) / (attempts - a);
            if (budget <= 0.005) break;
            setup.clear();
            const ob::PlannerStatus status = setup.solve(budget);
            last_status = status.asString();
            if (status != ob::PlannerStatus::EXACT_SOLUTION) continue;
            if (options_.simplify) setup.simplifySolution();

            std::vector<Joints> path;
            for (const ob::State* state : setup.getSolutionPath().getStates()) {
                const double* v = state->as<ob::RealVectorStateSpace::StateType>()->values;
                Joints q;
                for (int i = 0; i < 6; ++i) q[i] = v[i] / kDeg;
                path.push_back(q);
            }
            // the simplifier may move states by rounding error: pin the ends to what was asked
            path.front() = start;
            path.back() = goal;
            ++result.attempts;
            if (result.waypoints.empty() || travel(path) < travel(result.waypoints)) result.waypoints = std::move(path);
        }
        result.planning_time = elapsed();
        if (result.waypoints.empty()) {
            result.message = "no path found in " + std::to_string(options_.timeout) + " s (" + last_status + ")";
            return result;
        }
    }
    result.travel = travel(result.waypoints);
    result.solved = true;
    result.message = std::to_string(result.waypoints.size()) + " waypoints";
    return result;
}

Trajectory Planner::time_parameterize(const std::vector<Joints>& waypoints, const Limits& limits, double dt) const {
    Trajectory traj;
    if (waypoints.empty()) return traj;
    traj.times.push_back(0);
    traj.points.push_back(waypoints.front());
    double t0 = 0;

    for (std::size_t w = 1; w < waypoints.size(); ++w) {
        const Joints& a = waypoints[w - 1];
        const Joints& b = waypoints[w];
        // Path parameter s in [0, 1]: q = a + s (b - a). Joint j moves |b_j - a_j| per unit s,
        // so its limits become limits on ds/dt and d2s/dt2; the strictest joint wins.
        double v = std::numeric_limits<double>::infinity();
        double acc = std::numeric_limits<double>::infinity();
        for (int j = 0; j < 6; ++j) {
            const double d = std::abs(b[j] - a[j]);
            if (d < 1e-9) continue;
            v = std::min(v, limits.max_velocity[j] / d);
            acc = std::min(acc, limits.max_acceleration[j] / d);
        }
        if (!std::isfinite(v)) continue;   // repeated waypoint

        double ta = v / acc;               // time to reach cruise speed
        double T;
        if (v * ta >= 1.0) {               // never reaches v: triangle profile
            ta = std::sqrt(1.0 / acc);
            v = acc * ta;
            T = 2 * ta;
        } else {
            T = ta + 1.0 / v;
        }
        auto s_at = [&](double t) {
            if (t <= ta) return 0.5 * acc * t * t;
            if (t >= T - ta) return 1.0 - 0.5 * acc * (T - t) * (T - t);
            return 0.5 * acc * ta * ta + v * (t - ta);
        };

        const int n = std::max(1, static_cast<int>(std::ceil(T / dt)));
        for (int k = 1; k <= n; ++k) {
            const double t = T * k / n;
            const double s = k == n ? 1.0 : s_at(t);
            Joints q;
            for (int j = 0; j < 6; ++j) q[j] = a[j] + s * (b[j] - a[j]);
            traj.times.push_back(t0 + t);
            traj.points.push_back(q);
        }
        t0 += T;
    }
    return traj;
}

std::optional<std::string> Planner::verify(const Trajectory& trajectory) const {
    char prefix[64];
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
        const Joints& q = trajectory.points[i];
        // the sample itself, and the straight motion from the previous one
        const int n = i == 0 ? 1
                             : std::max(1, static_cast<int>(std::ceil(
                                               max_abs_diff(trajectory.points[i - 1], q) / options_.check_step_deg)));
        for (int k = 1; k <= n; ++k) {
            Joints p = q;
            if (i > 0) {
                const double s = static_cast<double>(k) / n;
                for (int j = 0; j < 6; ++j) p[j] = trajectory.points[i - 1][j] + s * (q[j] - trajectory.points[i - 1][j]);
            }
            if (auto why = check(p)) {
                std::snprintf(prefix, sizeof prefix, "t = %.3f s: ", trajectory.times[i]);
                return prefix + *why;
            }
        }
    }
    return std::nullopt;
}

}  // namespace ec::planning
