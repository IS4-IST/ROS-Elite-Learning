#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

// One MJPEG stream from one ESP32-CAM, published on /<name>/image_raw
class EspCamera
{
public:
  EspCamera(rclcpp::Node & node, std::string name, std::string url)
  : logger_(node.get_logger()),
    clock_(node.get_clock()),
    name_(std::move(name)),
    url_(std::move(url))
  {
    publisher_ = node.create_publisher<sensor_msgs::msg::Image>("/" + name_ + "/image_raw", 10);
    worker_ = std::thread(&EspCamera::loop, this);
  }

  ~EspCamera()
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void loop()
  {
    cv::VideoCapture cap(url_);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(logger_, "[%s] cannot open stream %s", name_.c_str(), url_.c_str());
      return;
    }
    RCLCPP_INFO(logger_, "[%s] streaming from %s", name_.c_str(), url_.c_str());

    cv::Mat frame;
    while (running_ && rclcpp::ok()) {
      if (!cap.read(frame) || frame.empty()) {
        RCLCPP_WARN(logger_, "[%s] failed to read frame", name_.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      std_msgs::msg::Header header;
      header.stamp = clock_->now();
      header.frame_id = name_;

      publisher_->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
    }
  }

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string url_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::thread worker_;
  std::atomic<bool> running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("esp_camera_bridge");

  auto names = node->declare_parameter<std::vector<std::string>>(
    "camera_names", std::vector<std::string>{"esp1", "esp2"});
  auto urls = node->declare_parameter<std::vector<std::string>>(
    "stream_urls",
    std::vector<std::string>{"****/stream", "****/stream"});

  if (names.size() != urls.size()) {
    RCLCPP_FATAL(node->get_logger(), "camera_names and stream_urls must have the same length");
    rclcpp::shutdown();
    return 1;
  }

  // Declared after node so they are destroyed before it
  std::vector<std::unique_ptr<EspCamera>> cameras;
  for (size_t i = 0; i < names.size(); ++i) {
    cameras.push_back(std::make_unique<EspCamera>(*node, names[i], urls[i]));
  }

  rclcpp::spin(node);
  cameras.clear();
  rclcpp::shutdown();
  return 0;
}
