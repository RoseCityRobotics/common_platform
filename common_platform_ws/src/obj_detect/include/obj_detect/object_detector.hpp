#ifndef OBJ_DETECT_OBJECT_DETECTOR_HPP
#define OBJ_DETECT_OBJECT_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include "obj_detect/msg/detection.hpp"
#include "obj_detect/msg/detection_array.hpp"

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

namespace obj_detect
{

struct Detection
{
  std::string class_name;
  float confidence;
  cv::Rect bbox;
};

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector();
  ~ObjectDetector();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void initialize_hailo();
  std::vector<Detection> run_inference(const cv::Mat& image);
  void publish_detections(const std::vector<Detection>& detections, const std_msgs::msg::Header& header);
  
#ifdef HAVE_HAILORT
  std::vector<Detection> parse_yolo_output(const std::vector<uint8_t>& output, 
                                           const hailo_vstream_info_t& shape,
                                           int orig_width, int orig_height);
#endif

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<obj_detect::msg::DetectionArray>::SharedPtr detection_publisher_;

  // HailoRT components
#ifdef HAVE_HAILORT
  std::shared_ptr<hailort::Device> device_;
  std::shared_ptr<hailort::ConfiguredNetworkGroup> configured_network_group_;
  std::vector<std::shared_ptr<hailort::InputVStream>> input_vstreams_;
  std::vector<std::shared_ptr<hailort::OutputVStream>> output_vstreams_;
#endif

  // Model parameters
  std::string model_path_;
  std::vector<std::string> class_names_;
  float confidence_threshold_;
  float nms_threshold_;
  int input_width_;
  int input_height_;
};

} // namespace obj_detect

#endif // OBJ_DETECT_OBJECT_DETECTOR_HPP
