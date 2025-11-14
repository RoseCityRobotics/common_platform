#ifndef IMITATION_LEARNING_IMITATION_LEARNING_NODE_HPP
#define IMITATION_LEARNING_IMITATION_LEARNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>

#ifdef HAVE_HAILORT
#include <hailo/hailort.hpp>
#include <hailo/device.hpp>
#include <hailo/vstream.hpp>
#include <hailo/network_group.hpp>
#include <hailo/hef.hpp>
#endif

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <mutex>

namespace imitation_learning
{

class ImitationLearningNode : public rclcpp::Node
{
public:
  explicit ImitationLearningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ImitationLearningNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publish_cmd_vel_timer_callback();
  void initialize_hailo();
  std::vector<float> run_inference(const cv::Mat& image);
  void preprocess_image(const cv::Mat& image, cv::Mat& output);

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // HailoRT components
#ifdef HAVE_HAILORT
  std::shared_ptr<hailort::Device> device_;
  std::shared_ptr<hailort::ConfiguredNetworkGroup> configured_network_group_;
  std::unique_ptr<hailort::ActivatedNetworkGroup> activated_network_group_;
  std::vector<std::shared_ptr<hailort::InputVStream>> input_vstreams_;
  std::vector<std::shared_ptr<hailort::OutputVStream>> output_vstreams_;
#endif

  // Model parameters
  std::string model_path_;
  int input_width_;
  int input_height_;
  int sequence_length_;
  float max_linear_velocity_;
  float max_angular_velocity_;

  // Image buffer for sequence processing
  std::deque<cv::Mat> image_buffer_;
  std::mutex buffer_mutex_;

  // Latest prediction
  std::mutex prediction_mutex_;
  float latest_linear_x_;
  float latest_angular_z_;
  bool has_prediction_;

  // Publishing rate
  double publish_rate_;
};

} // namespace imitation_learning

#endif // IMITATION_LEARNING_IMITATION_LEARNING_NODE_HPP

