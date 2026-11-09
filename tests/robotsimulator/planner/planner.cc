// Collision-free joint motion planning in the MuJoCo scene (OMPL), sent to the digital
// twin as a ROS 2 action goal. The planner never talks to the robot: the twin is the
// only commander, and it re-checks everything before anything moves.
//
//   ./planner --goal J1 J2 J3 J4 J5 J6 [options]           start = /joint_states (from the twin)
//   ./planner --goal ... --execute                          send it to the twin
//   ./planner --start home --goal ...                       offline: plan and print only
//
//   --goal Q|home      target joints, deg (or the model's "home" keyframe)
//   --start Q|home     plan from these joints instead of /joint_states (can't be executed)
//   --model PATH       scene to plan in (default library/models/scene.xml); give the twin the same
//   --algorithm NAME   RRTConnect (default), RRT, RRTstar, PRM, KPIECE1, BKPIECE1
//   --timeout S        search time, default 2 (shared by the attempts)
//   --attempts N       independent searches, the shortest path wins (default 8)
//   --margin MM        minimum clearance of the moving links, default 10
//   --execute          send the motion to the twin (/follow_joint_trajectory)
//   --timed            send dense samples timed HERE (--speed, --acc); default: send the path's
//                      waypoints only and let the twin time them with its own limits
//   --speed DEG_S      joint speed for --timed and for the duration estimate, default 30
//   --acc DEG_S2       joint acceleration, same, default 60
//
// The twin accepts motions only after its operator ticks "accept external commands";
// otherwise the goal comes back aborted with the reason. Ctrl+C during execution cancels
// the goal, which stops the arm.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>

#include "common.h"
#include "planning/planner.h"

#ifdef EC_HAVE_ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ros_env.h"
#endif

using namespace std::chrono_literals;

namespace {

constexpr double kDeg = mjPI / 180.0;
std::atomic<bool> interrupted{false};

void print_joints(const char* label, const ec::Joints& q) {
    std::printf("%-8s [", label);
    for (int i = 0; i < 6; ++i) std::printf("%8.2f%s", q[i], i < 5 ? "," : "");
    std::printf("]\n");
}

// "home" (the model's keyframe) or six numbers starting at argv[i]
ec::Joints parse_joints(int argc, char** argv, int& i, const mjModel* m) {
    ec::Joints q{};
    if (i + 1 < argc && std::string(argv[i + 1]) == "home") {
        ++i;
        const int key = mj_name2id(m, mjOBJ_KEY, "home");
        if (key < 0) throw std::runtime_error("the model has no 'home' keyframe");
        for (int j = 0; j < 6; ++j) q[j] = m->key_qpos[key * m->nq + j] / kDeg;
        return q;
    }
    if (i + 6 >= argc) throw std::runtime_error(std::string(argv[i]) + " needs 6 joint values (deg) or 'home'");
    for (int j = 0; j < 6; ++j) q[j] = std::stod(argv[++i]);
    return q;
}

#ifdef EC_HAVE_ROS
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
const char* kJointNames[6] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

// Current joints from the twin's /joint_states (deg)
ec::Joints read_joint_states(const rclcpp::Node::SharedPtr& node, rclcpp::Executor& executor) {
    std::optional<ec::Joints> q;
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [&q](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
            ec::Joints v{};
            int found = 0;
            for (std::size_t c = 0; c < msg->name.size() && c < msg->position.size(); ++c)
                for (int j = 0; j < 6; ++j)
                    if (msg->name[c] == kJointNames[j]) { v[j] = msg->position[c] / kDeg; ++found; }
            if (found == 6) q = v;
        });
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!q && std::chrono::steady_clock::now() < deadline && !interrupted) executor.spin_once(50ms);
    if (!q) throw std::runtime_error("no /joint_states in 3 s: is the digital twin running (with ROS)? "
                                     "Or plan offline with --start");
    return *q;
}

// Send the motion, print feedback, return the exit code
int execute(const rclcpp::Node::SharedPtr& node, rclcpp::Executor& executor, const std::vector<ec::Joints>& waypoints,
            const ec::planning::Trajectory& traj, bool timed) {
    auto client = rclcpp_action::create_client<FollowJointTrajectory>(node, "follow_joint_trajectory");
    if (!client->wait_for_action_server(3s))
        throw std::runtime_error("no /follow_joint_trajectory: is the digital twin running (with ROS)?");

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names.assign(std::begin(kJointNames), std::end(kJointNames));
    auto add = [&goal](const ec::Joints& q, double t) {
        trajectory_msgs::msg::JointTrajectoryPoint p;
        for (int j = 0; j < 6; ++j) p.positions.push_back(q[j] * kDeg);
        p.time_from_start = rclcpp::Duration::from_seconds(t);
        goal.trajectory.points.push_back(p);
    };
    if (timed) {
        for (std::size_t i = 0; i < traj.points.size(); ++i) add(traj.points[i], traj.times[i]);
    } else {
        for (const auto& q : waypoints) add(q, 0.0);   // all times 0 = a path: the twin times it
    }
    std::printf("sending  %zu %s to the twin\n", goal.trajectory.points.size(),
                timed ? "timed samples (timed here)" : "waypoints (the twin times them)");

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    auto last_print = std::chrono::steady_clock::now();
    options.feedback_callback = [&last_print](auto, const std::shared_ptr<const FollowJointTrajectory::Feedback> fb) {
        if (std::chrono::steady_clock::now() - last_print < 500ms) return;
        last_print = std::chrono::steady_clock::now();
        double err = 0;
        for (double e : fb->error.positions) err = std::max(err, std::abs(e) / kDeg);
        std::printf("  t = %5.2f s  tracking error %.2f deg\n", rclcpp::Duration(fb->desired.time_from_start).seconds(), err);
    };
    auto goal_future = client->async_send_goal(goal, options);
    if (executor.spin_until_future_complete(goal_future, 5s) != rclcpp::FutureReturnCode::SUCCESS || !goal_future.get())
        throw std::runtime_error("the twin did not accept the goal");
    auto handle = goal_future.get();
    auto result_future = client->async_get_result(handle);

    bool cancel_sent = false;
    while (executor.spin_until_future_complete(result_future, 50ms) == rclcpp::FutureReturnCode::TIMEOUT) {
        if (interrupted && !cancel_sent) {
            std::printf("\ncancelling (the twin stops the arm)\n");
            client->async_cancel_goal(handle);
            cancel_sent = true;
        }
    }
    const auto wrapped = result_future.get();
    const std::string text = wrapped.result ? wrapped.result->error_string : "";
    switch (wrapped.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            std::printf("done     %s\n", text.c_str());
            return 0;
        case rclcpp_action::ResultCode::CANCELED:
            std::printf("canceled %s\n", text.c_str());
            return 130;
        default:
            std::printf("FAILED   %s\n", text.c_str());
            return 3;
    }
}
#endif

}  // namespace

int main(int argc, char** argv) {
#ifdef EC_HAVE_ROS
    ec::ensure_ros_environment(argv);   // first: may restart the program once
#endif
    try {
        // the model first: --start/--goal home need it
        std::string model_path = ec::default_model_path();
        for (int i = 1; i + 1 < argc; ++i)
            if (std::string(argv[i]) == "--model") model_path = argv[i + 1];
        ec::ModelPtr model = ec::load_model(model_path);

        std::optional<ec::Joints> start, goal;
        ec::planning::Options options;
        ec::planning::Limits limits;
        double speed = 30, acc = 60;
        bool execute_it = false, timed = false;
        for (int i = 1; i < argc; ++i) {
            const std::string a = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
                return argv[++i];
            };
            if (a == "--model") ++i;
            else if (a == "--start") start = parse_joints(argc, argv, i, model.get());
            else if (a == "--goal") goal = parse_joints(argc, argv, i, model.get());
            else if (a == "--algorithm") options.algorithm = next();
            else if (a == "--timeout") options.timeout = std::stod(next());
            else if (a == "--attempts") options.attempts = std::stoi(next());
            else if (a == "--margin") options.margin = std::stod(next()) / 1000.0;
            else if (a == "--speed") speed = std::stod(next());
            else if (a == "--acc") acc = std::stod(next());
            else if (a == "--execute") execute_it = true;
            else if (a == "--timed") timed = true;
            else if (a == "--ros-args") break;   // the rest belongs to ROS
            else throw std::runtime_error("unknown option " + a);
        }
        if (!goal) throw std::runtime_error("--goal is required (6 joint values in deg, or 'home')");
        if (execute_it && start) throw std::runtime_error("--execute starts from the robot's joints: drop --start");
        limits.max_velocity.fill(speed);
        limits.max_acceleration.fill(acc);
        std::signal(SIGINT, [](int) { interrupted = true; });

#ifdef EC_HAVE_ROS
        // No ROS signal handler: Ctrl+C sets `interrupted`, which cancels a running goal
        rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
        auto node = std::make_shared<rclcpp::Node>("planner");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        struct Shutdown { ~Shutdown() { rclcpp::shutdown(); } } shutdown_at_exit;
        if (!start) start = read_joint_states(node, executor);
#else
        if (execute_it) throw std::runtime_error("--execute needs ROS 2: this planner was built without it");
        if (!start) throw std::runtime_error("built without ROS 2: give --start");
#endif

        ec::planning::Planner planner(model.get(), options);
        std::printf("scene    %s\nplanner  %s, timeout %.1f s, margin %.0f mm\n", model_path.c_str(),
                    options.algorithm.c_str(), options.timeout, options.margin * 1000);
        for (const std::string& pair : planner.relaxed_pairs())
            std::printf("relaxed  %s (close in every pose: only penetration is checked)\n", pair.c_str());
        print_joints("start", *start);
        print_joints("goal", *goal);

        // Would the plain joint move have worked? (useful to know, and a sanity check)
        {
            const ec::planning::Trajectory straight = planner.time_parameterize({*start, *goal}, limits);
            const auto why = planner.verify(straight);
            double closest = 1e9;
            for (const ec::Joints& q : straight.points) closest = std::min(closest, planner.clearance(q));
            if (why)
                std::printf("direct   blocked, first at %s; closest %.1f mm%s\n", why->c_str(), closest * 1000,
                            closest < 0 ? " = COLLISION" : "");
            else
                std::printf("direct   free (a plain moveByJoint would do)\n");
        }

        // --- plan, time, verify -----------------------------------------------------------
        const ec::planning::Result result = planner.plan(*start, *goal);
        if (!result.solved) {
            std::printf("FAILED   %s\n", result.message.c_str());
            return 2;
        }
        const ec::planning::Trajectory traj = planner.time_parameterize(result.waypoints, limits);
        if (auto why = planner.verify(traj)) {
            std::printf("FAILED   timed trajectory not collision-free: %s\n", why->c_str());
            return 2;
        }
        double min_clearance = 1e9;
        for (std::size_t i = 0; i < traj.points.size(); i += 5)
            min_clearance = std::min(min_clearance, planner.clearance(traj.points[i]));

        if (result.attempts == 0)
            std::printf("planned  straight line (no search needed)\n");
        else
            std::printf("planned  %zu waypoints, best of %d paths in %.3f s, joint travel %.0f deg\n",
                        result.waypoints.size(), result.attempts, result.planning_time, result.travel);
        for (std::size_t i = 0; i < result.waypoints.size(); ++i)
            print_joints(("  " + std::to_string(i)).c_str(), result.waypoints[i]);
        std::printf("timed    %.2f s, %zu points at %.0f deg/s, %.0f deg/s^2%s\n", traj.duration(), traj.points.size(),
                    speed, acc, timed || !execute_it ? "" : " (estimate: the twin times it with its own limits)");
        std::printf("clear    closest approach %.1f mm%s\n", min_clearance * 1000,
                    min_clearance >= options.margin + 0.049 ? " or more" : "");

        if (!execute_it) {
#ifdef EC_HAVE_ROS
            std::printf("dry run: nothing sent (add --execute to send it to the twin)\n");
#else
            std::printf("dry run (built without ROS 2: this planner can't send motions)\n");
#endif
            return 0;
        }
#ifdef EC_HAVE_ROS
        return execute(node, executor, result.waypoints, traj, timed);
#endif
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
