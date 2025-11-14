#ifndef IMITATION_LEARNING_IMITATION_LEARNING_NODE_HPP
#define IMITATION_LEARNING_IMITATION_LEARNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <mutex>
#include <atomic>

namespace imitation_learning
{

struct InferenceStats
{
  std::atomic<uint64_t> total_inferences{0};
  std::atomic<double> total_inference_time_ms{0.0};
  std::atomic<double> min_inference_time_ms{1000.0};
  std::atomic<double> max_inference_time_ms{0.0};
  std::atomic<uint64_t> dropped_frames{0};
  std::chrono::steady_clock::time_point last_stats_report;
};

class ImitationLearningNode : public rclcpp::Node
{
public:
  explicit ImitationLearningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ImitationLearningNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publish_cmd_vel_timer_callback();
  void stats_report_timer_callback();
  void initialize_onnx();
  std::vector<float> run_inference(const cv::Mat& image);
  void preprocess_image(const cv::Mat& image, cv::Mat& output);

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // ONNX Runtime components
#ifdef HAVE_ONNXRUNTIME
  Ort::Env ort_env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::AllocatorWithDefaultOptions allocator_;
  std::vector<std::string> input_name_strings_;  // Keep names alive
  std::vector<std::string> output_name_strings_;  // Keep names alive
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;
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

  // Performance monitoring
  InferenceStats stats_;
  std::mutex inference_mutex_;
  bool inference_in_progress_;
  
  // Timing
  std::chrono::steady_clock::time_point last_image_time_;
  std::chrono::steady_clock::time_point last_inference_start_;
};

} // namespace imitation_learning

#endif // IMITATION_LEARNING_IMITATION_LEARNING_NODE_HPP

