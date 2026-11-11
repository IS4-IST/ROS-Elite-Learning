// The twin's ROS 2 front end (built only when ROS 2 is found; see CMakeLists.txt).
//
// Node "digitaltwin":
//   publishers   /joint_states    sensor_msgs/JointState         every monitor packet (125 Hz), rad
//                /robot_status    diagnostic_msgs/DiagnosticStatus  10 Hz: mode, state, servo,
//                                 e-stop, 8055, armed, external_control, active_motion, ...
//                (camera: later)
//   services     /stop            std_srvs/Trigger   anyone, always
//                /power           std_srvs/SetBool   needs the operator's external grant
//                /clear_alarm     std_srvs/Trigger   needs the operator's external grant
//   action       /follow_joint_trajectory   control_msgs/action/FollowJointTrajectory
//
// The action takes either
//   - a PATH: every time_from_start is 0. The twin times it with its own motion limits.
//   - a TIMED trajectory: times increase. The twin checks speed and acceleration against
//     its limits on the 8 ms samples it will send (sparse points = velocity steps = refused).
// If the first point isn't the current joint state, the current state is put in front
// (time 0). Joint names: joint1..joint6 in any order, or empty for that order.
// Everything is collision-checked in the twin's scene before it is sent.
//
// Every goal is accepted, then checked: refusals (no external grant, robot moving, collision,
// limits) come back as an ABORTED result with the reason in error_string, so the client
// sees why. Cancelling the goal stops the arm.
#pragma once

#include <memory>

#include "twin_core.h"

namespace twin {

class RosInterface {
public:
    RosInterface(Core& core, int argc, char** argv);   // rclcpp::init + node + executor thread
    ~RosInterface();                                    // rclcpp::shutdown + join
    RosInterface(const RosInterface&) = delete;
    RosInterface& operator=(const RosInterface&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace twin
