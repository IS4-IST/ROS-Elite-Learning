// The twin's ROS 2 front end (built only when ROS 2 is found; see CMakeLists.txt).
//
// Node "digitaltwin":
//   publishers   /joint_states    sensor_msgs/JointState         every monitor packet (125 Hz), rad,
//                                 stamped with the packet's arrival time (system clock)
//                /robot_status    diagnostic_msgs/DiagnosticStatus  10 Hz: mode, state, servo,
//                                 e-stop, 8055, armed, external_control, active_motion, stream_*
//                /cameras/<name>/image_raw  sensor_msgs/Image, rgb8, every decoded frame of each
//                                 --camera, stamped with its CAPTURE time (from the stream's SEI),
//                                 best-effort QoS (subscribe with the sensor-data profile)
//   subscription /joint_command   sensor_msgs/JointState: one joint target (rad) per message, for
//                                 a policy or teleop. It starts and feeds a stream (Core::stream_target):
//                                 needs the operator's external grant, every target is checked,
//                                 and silence (300 ms by default) ends the stream. Best-effort QoS.
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
#include <string>
#include <vector>

#include "camera/client.h"
#include "twin_core.h"

namespace twin {

class RosInterface {
public:
    // rclcpp::init + node + executor thread. One image publisher per name in cameras.
    RosInterface(Core& core, const std::vector<std::string>& cameras, int argc, char** argv);
    ~RosInterface();                                    // rclcpp::shutdown + join
    RosInterface(const RosInterface&) = delete;
    RosInterface& operator=(const RosInterface&) = delete;

    // Publishes a decoded camera frame on /cameras/<name>/image_raw (a name given to the
    // constructor; others are ignored). Any thread: it only reads the publisher map.
    void publish_camera(const std::string& name, const camera::DecodedFrame& image, const camera::FrameStamp& stamp);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace twin
