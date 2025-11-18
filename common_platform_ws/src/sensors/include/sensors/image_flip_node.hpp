#ifndef SENSORS_IMAGE_FLIP_NODE_HPP
#define SENSORS_IMAGE_FLIP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

namespace sensors
{

class ImageFlipNode : public rclcpp::Node
{
public:
  explicit ImageFlipNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  // Parameters
  bool flip_enabled_;
  int flip_code_;  // 0 = vertical, 1 = horizontal, -1 = both
  std::string input_topic_;
  std::string output_topic_;
};

}  // namespace sensors

#endif  // SENSORS_IMAGE_FLIP_NODE_HPP

