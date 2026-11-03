#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "esp_camera_bridge/esp_udp_client.hpp"

// Receives JPEG frames from one ESP32-CAM over UDP and publishes them on /<camera_name>/image_raw
class EspCameraNode : public rclcpp::Node
{
public:
  EspCameraNode()
  : Node("esp_camera_bridge")
  {
    camera_name_ = declare_parameter<std::string>("camera_name", "esp1");
    esp_ip_ = declare_parameter<std::string>("esp_ip", "192.168.4.1");
    command_port_ = static_cast<uint16_t>(declare_parameter<int>("command_port", 3333));
    udp_port_ = static_cast<uint16_t>(declare_parameter<int>("udp_port", 3334));
    mode_ = declare_parameter<std::string>("mode", "stream");

    publisher_ = create_publisher<sensor_msgs::msg::Image>("/" + camera_name_ + "/image_raw", 10);

    client_ = std::make_unique<esp_camera_bridge::EspUdpClient>(esp_ip_, command_port_, udp_port_);
    client_->start([this](std::vector<uint8_t> && jpeg) { onFrame(std::move(jpeg)); });
    client_->sendCommand(mode_);

    RCLCPP_INFO(
      get_logger(), "[%s] %s mode, commands to %s:%u, frames on UDP %u",
      camera_name_.c_str(), mode_.c_str(), esp_ip_.c_str(), command_port_, udp_port_);
  }

  ~EspCameraNode() override
  {
    // Joins the receive thread before the publisher goes away
    client_->stop();
  }

private:
  void onFrame(std::vector<uint8_t> && jpeg)
  {
    cv::Mat frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "[%s] dropped a frame that is not a valid JPEG", camera_name_.c_str());
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = camera_name_;

    publisher_->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
  }

  std::string camera_name_;
  std::string esp_ip_;
  uint16_t command_port_{3333};
  uint16_t udp_port_{3334};
  std::string mode_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::unique_ptr<esp_camera_bridge::EspUdpClient> client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<EspCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("esp_camera_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
