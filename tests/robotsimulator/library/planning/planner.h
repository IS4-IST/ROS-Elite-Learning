// Joint-space motion planning for the EC66: OMPL for the search, MuJoCo for collisions.
//
//   ec::planning::Planner planner(model);              // private copy of the model
//   if (auto why = planner.check(goal)) ...             // goal in collision / out of range
//   auto result = planner.plan(start, goal);            // waypoints, deg
//   auto traj = planner.time_parameterize(result.waypoints, limits);
//   if (auto why = planner.verify(traj)) ...            // every sample re-checked
//   robot.run_joint_trajectory(traj.times, traj.points);
//
// Collision model: the scene's collision geoms (group 3 meshes for the arm, as their
// convex hulls, plus the floor and anything else you add to the scene). A configuration
// is valid when every moving arm link stays at least `margin` away from everything else,
// including the arm's own non-adjacent links. Contacts that don't involve a moving arm
// link (base on floor, a box resting on a table) are ignored.
//
// Some pairs of the arm's own links are close in EVERY pose (the shoulder housing of
// link2 sits ~14 mm from the base). Holding them to the margin would make every pose
// invalid, so at construction 500 random poses are sampled, as MoveIt's setup assistant
// does; pairs within the margin in all of them are only required not to penetrate.
// relaxed_pairs() lists them.
//
// All angles in degrees, like the controller. Not thread-safe: one Planner per thread.
#pragma once

#include <mujoco/mujoco.h>

#include <optional>
#include <string>
#include <vector>

#include "ec_robot/types.hpp"   // ec::Joints (header only)

namespace ec::planning {

struct Options {
    std::string algorithm = "RRTConnect";   // RRTConnect, RRT, RRTstar, PRM, KPIECE1, BKPIECE1
    double timeout = 2.0;                   // s, total for all attempts
    int attempts = 8;                       // independent searches; the shortest path wins
    double margin = 0.01;                   // m, minimum clearance of the moving links
    double check_step_deg = 1.0;            // largest joint step between collision checks
    bool simplify = true;                   // shortcut + smooth the raw path
};

struct Limits {
    Joints max_velocity{60, 60, 60, 90, 90, 90};       // deg/s
    Joints max_acceleration{120, 120, 120, 180, 180, 180};   // deg/s^2
};

struct Result {
    bool solved = false;
    std::vector<Joints> waypoints;   // start ... goal
    double planning_time = 0;        // s
    int attempts = 0;                // searches that found a path
    double travel = 0;               // deg: sum over segments of the largest joint change
    std::string message;
};

struct Trajectory {
    std::vector<double> times;       // s, from 0
    std::vector<Joints> points;      // deg
    double duration() const { return times.empty() ? 0 : times.back(); }
};

class Planner {
public:
    explicit Planner(const mjModel* model, Options options = {});
    ~Planner();
    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;

    // Copy the positions of everything that isn't the arm (free objects, other robots)
    // from a live simulation, so the planner sees obstacles where they are now.
    void update_scene(const mjData* d);

    // nullopt if q is valid; otherwise why not ("joint 2 out of range", "link3 / floor: 4.2 mm")
    std::optional<std::string> check(const Joints& q) const;

    // Smallest clearance (m) between a moving arm link and anything else, within `margin`
    // + 5 cm (beyond that it reports that bound). Negative = penetration.
    double clearance(const Joints& q) const;

    // RRT-style planners return a different, often roundabout, path every run. plan()
    // runs `attempts` searches (sharing the timeout) and keeps the one with the least
    // joint travel, so wrists don't swing around for a sideways move.
    Result plan(const Joints& start, const Joints& goal) const;

    // Stop-at-every-waypoint profile: each segment is a straight line in joint space
    // with a trapezoidal speed profile, timed so no joint exceeds its limits.
    // Sampled every `dt` seconds, endpoints included.
    Trajectory time_parameterize(const std::vector<Joints>& waypoints, const Limits& limits,
                                 double dt = 0.008) const;

    // Re-check every sample, and the straight motion between consecutive samples.
    std::optional<std::string> verify(const Trajectory& trajectory) const;

    const Options& options() const { return options_; }
    // arm link pairs that are only checked for penetration (see top of file)
    std::vector<std::string> relaxed_pairs() const;
    double lower(int joint) const { return lower_[joint]; }   // deg
    double upper(int joint) const { return upper_[joint]; }   // deg

private:
    // slack = distance - required distance (margin, or 0 for relaxed pairs); valid iff slack >= 0
    struct Worst { double slack, distance; int geom1, geom2; };
    Worst worst_contact(const Joints& q) const;   // over contacts involving a moving arm link
    void find_relaxed_pairs();
    bool relaxed(int g1, int g2) const;
    void set_arm(const Joints& q) const;

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    Options options_;
    int qpos_adr_[6];
    double lower_[6], upper_[6];       // deg
    std::vector<char> moving_arm_;     // per body: part of the arm and moved by its joints
    std::vector<char> arm_;            // per body: part of the arm, base included
    std::vector<std::pair<int, int>> relaxed_;   // geom pairs, first < second
};

}  // namespace ec::planning
