// station_bridge: publishes the Elite CS robot state (RTSI) and the ESP32-CAM stream into ROS 2.
//
// Threads:
//   - RTSI thread (RtsiReader): blocks in receiveData(); publishes each robot sample as it arrives.
//   - camera thread (asio::io_context): publishes each complete JPEG frame as it arrives.
//   - main thread: rclcpp::spin (parameters, services, diagnostics timer).
// rclcpp publishers are thread-safe, so both threads publish directly; no queues in between.
//
// Topics (relative names, so they follow the node's namespace):
//   joint_states                    sensor_msgs/JointState      actual q, qd, tau
//   ~/joint_targets                 sensor_msgs/JointState      target q
//   ~/tcp_pose                      geometry_msgs/PoseStamped   in base_frame
//   ~/tcp_twist                     geometry_msgs/TwistStamped
//   ~/tcp_wrench                    geometry_msgs/WrenchStamped
//   ~/speed_scaling                 std_msgs/Float64
//   ~/robot_mode, ~/safety_mode     std_msgs/Int32 (latched)
//   camera/image/compressed         sensor_msgs/CompressedImage format "jpeg", not decoded
#include "esp32cam/frame_receiver.hpp"
#include "robot/rtsi_reader.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

namespace {

// Elite TCP orientation is a rotation vector (axis * angle, rad).
geometry_msgs::msg::Quaternion rotvec_to_quaternion(double rx, double ry, double rz)
{
    geometry_msgs::msg::Quaternion q;
    const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (angle < 1e-12) {
        q.w = 1.0;
        return q;
    }
    const double s = std::sin(angle / 2.0) / angle;
    q.x = rx * s;
    q.y = ry * s;
    q.z = rz * s;
    q.w = std::cos(angle / 2.0);
    return q;
}

}  // namespace

class StationBridge : public rclcpp::Node {
public:
    explicit StationBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("station_bridge", options)
    {
        // ---- parameters
        robot_ip_       = declare_parameter<std::string>("robot_ip", "192.168.1.200");
        rtsi_frequency_ = declare_parameter<double>("rtsi_frequency", 250.0);
        joint_names_    = declare_parameter<std::vector<std::string>>(
            "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
        base_frame_     = declare_parameter<std::string>("base_frame", "base_link");

        enable_camera_  = declare_parameter<bool>("camera.enabled", true);
        camera_ip_      = declare_parameter<std::string>("camera.ip", "192.168.4.1");
        camera_tcp_port_ = declare_parameter<int>("camera.tcp_port", 3333);
        camera_udp_port_ = declare_parameter<int>("camera.udp_port", 3334);
        camera_frame_   = declare_parameter<std::string>("camera.frame_id", "camera_optical_frame");
        const bool enable_robot = declare_parameter<bool>("robot.enabled", true);

        if (joint_names_.size() != 6)
            throw std::invalid_argument("joint_names must have exactly 6 entries");

        // ---- publishers
        // joint_states: default (reliable) QoS so robot_state_publisher & co. accept it.
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_target_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_targets", 10);
        tcp_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tcp_pose", 10);
        tcp_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/tcp_twist", 10);
        tcp_wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/tcp_wrench", 10);
        speed_scaling_pub_ = create_publisher<std_msgs::msg::Float64>("~/speed_scaling", 10);
        // Modes only change occasionally: publish on change, latched for late subscribers.
        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        robot_mode_pub_ = create_publisher<std_msgs::msg::Int32>("~/robot_mode", latched);
        safety_mode_pub_ = create_publisher<std_msgs::msg::Int32>("~/safety_mode", latched);
        // Images: best effort, keep only the newest. A slow subscriber gets fewer frames, never old ones.
        image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "camera/image/compressed", rclcpp::SensorDataQoS().keep_last(1));

        // ---- sources
        if (enable_robot) {
            rtsi_ = std::make_unique<robot::RtsiReader>(
                robot_ip_, rtsi_frequency_,
                [this](const robot::RobotSample& s) { publish_robot(s); },
                [this](bool ok, const std::string& msg) {
                    if (ok) RCLCPP_INFO(get_logger(), "robot: %s", msg.c_str());
                    else RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "robot: %s", msg.c_str());
                });
            rtsi_->start();
        }

        if (enable_camera_) {
            camera_ = std::make_unique<esp32cam::FrameReceiver>(
                io_, camera_ip_, static_cast<uint16_t>(camera_tcp_port_),
                static_cast<uint16_t>(camera_udp_port_),
                [this](esp32cam::Frame&& f) { publish_frame(std::move(f)); });
            camera_->start();
            io_thread_ = std::thread([this] {
                auto guard = asio::make_work_guard(io_);
                io_.run();
            });
        }

        status_timer_ = create_wall_timer(5s, [this] { report_status(); });
        RCLCPP_INFO(get_logger(), "robot %s @ %.0f Hz, camera %s:%d (udp %d)",
                    enable_robot ? robot_ip_.c_str() : "disabled", rtsi_frequency_,
                    enable_camera_ ? camera_ip_.c_str() : "disabled",
                    static_cast<int>(camera_tcp_port_), static_cast<int>(camera_udp_port_));
    }

    ~StationBridge() override
    {
        if (rtsi_) rtsi_->stop();
        if (camera_) camera_->stop();
        io_.stop();
        if (io_thread_.joinable()) io_thread_.join();
    }

private:
    // Called on the RTSI thread for every received packet.
    void publish_robot(const robot::RobotSample& s)
    {
        const auto stamp = now();
        ++robot_samples_;

        sensor_msgs::msg::JointState js;
        js.header.stamp = stamp;
        js.name = joint_names_;
        js.position.assign(s.q.begin(), s.q.end());
        js.velocity.assign(s.qd.begin(), s.qd.end());
        js.effort.assign(s.tau.begin(), s.tau.end());
        joint_state_pub_->publish(js);

        sensor_msgs::msg::JointState target;
        target.header.stamp = stamp;
        target.name = joint_names_;
        target.position.assign(s.q_target.begin(), s.q_target.end());
        joint_target_pub_->publish(target);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = base_frame_;
        pose.pose.position.x = s.tcp_pose[0];
        pose.pose.position.y = s.tcp_pose[1];
        pose.pose.position.z = s.tcp_pose[2];
        pose.pose.orientation = rotvec_to_quaternion(s.tcp_pose[3], s.tcp_pose[4], s.tcp_pose[5]);
        tcp_pose_pub_->publish(pose);

        geometry_msgs::msg::TwistStamped twist;
        twist.header = pose.header;
        twist.twist.linear.x = s.tcp_speed[0];
        twist.twist.linear.y = s.tcp_speed[1];
        twist.twist.linear.z = s.tcp_speed[2];
        twist.twist.angular.x = s.tcp_speed[3];
        twist.twist.angular.y = s.tcp_speed[4];
        twist.twist.angular.z = s.tcp_speed[5];
        tcp_twist_pub_->publish(twist);

        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header = pose.header;
        wrench.wrench.force.x = s.tcp_force[0];
        wrench.wrench.force.y = s.tcp_force[1];
        wrench.wrench.force.z = s.tcp_force[2];
        wrench.wrench.torque.x = s.tcp_force[3];
        wrench.wrench.torque.y = s.tcp_force[4];
        wrench.wrench.torque.z = s.tcp_force[5];
        tcp_wrench_pub_->publish(wrench);

        std_msgs::msg::Float64 scaling;
        scaling.data = s.speed_scaling;
        speed_scaling_pub_->publish(scaling);

        if (s.robot_mode != last_robot_mode_) {
            RCLCPP_INFO(get_logger(), "robot mode: %s", robot::robot_mode_name(s.robot_mode));
            std_msgs::msg::Int32 m;
            m.data = last_robot_mode_ = s.robot_mode;
            robot_mode_pub_->publish(m);
        }
        if (s.safety_mode != last_safety_mode_) {
            const char* name = robot::safety_mode_name(s.safety_mode);
            if (s.safety_mode == 1 || s.safety_mode == 2) RCLCPP_INFO(get_logger(), "safety mode: %s", name);
            else RCLCPP_WARN(get_logger(), "safety mode: %s", name);
            std_msgs::msg::Int32 m;
            m.data = last_safety_mode_ = s.safety_mode;
            safety_mode_pub_->publish(m);
        }
    }

    // Called on the camera io thread for every complete frame. The JPEG bytes are moved, not copied.
    void publish_frame(esp32cam::Frame&& f)
    {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = now();
        msg->header.frame_id = camera_frame_;
        msg->format = "jpeg";
        msg->data = std::move(f.jpeg);
        image_pub_->publish(std::move(msg));
        ++camera_frames_;
    }

    void report_status()
    {
        const auto t = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(t - last_report_).count();
        last_report_ = t;
        const uint64_t samples = robot_samples_.exchange(0), frames = camera_frames_.exchange(0);
        char cam[96] = "disabled";
        if (camera_) {
            const auto st = camera_->stats();
            std::snprintf(cam, sizeof cam, "%s%.1f fps, %llu dropped total",
                          camera_->connected() ? "" : "DISCONNECTED ", frames / dt,
                          static_cast<unsigned long long>(st.dropped));
        }
        char rob[64] = "disabled";
        if (rtsi_)
            std::snprintf(rob, sizeof rob, "%s%.0f Hz", rtsi_->connected() ? "" : "DISCONNECTED ", samples / dt);
        RCLCPP_INFO(get_logger(), "robot %s | camera %s", rob, cam);
    }

    // parameters
    std::string robot_ip_;
    double rtsi_frequency_;
    std::vector<std::string> joint_names_;
    std::string base_frame_;
    bool enable_camera_;
    std::string camera_ip_;
    int64_t camera_tcp_port_, camera_udp_port_;
    std::string camera_frame_;

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_, joint_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tcp_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tcp_wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_scaling_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_mode_pub_, safety_mode_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // state touched only by the RTSI thread
    int32_t last_robot_mode_ = -1000, last_safety_mode_ = -1000;

    // sources
    std::unique_ptr<robot::RtsiReader> rtsi_;
    asio::io_context io_;
    std::unique_ptr<esp32cam::FrameReceiver> camera_;
    std::thread io_thread_;

    std::atomic<uint64_t> robot_samples_{0}, camera_frames_{0};
    std::chrono::steady_clock::time_point last_report_ = std::chrono::steady_clock::now();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StationBridge>();
    rclcpp::spin(node);
    node.reset();      // stop the source threads before rclcpp shuts down
    rclcpp::shutdown();
    return 0;
}
