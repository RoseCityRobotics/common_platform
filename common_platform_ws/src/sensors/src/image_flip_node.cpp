#include "sensors/image_flip_node.hpp"
#include <sensor_msgs/image_encodings.hpp>

namespace sensors
{

ImageFlipNode::ImageFlipNode(const rclcpp::NodeOptions& options)
  : Node("image_flip_node", options)
{
  // Declare parameters
  this->declare_parameter<bool>("flip_enabled", true);
  this->declare_parameter<int>("flip_code", -1);  // -1 = both axes (180° rotation)
  this->declare_parameter<std::string>("input_topic", "camera/image_raw");
  this->declare_parameter<std::string>("output_topic", "camera/image_raw");

  // Get parameters
  flip_enabled_ = this->get_parameter("flip_enabled").as_bool();
  flip_code_ = this->get_parameter("flip_code").as_int();
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();

  // Validate flip_code
  if (flip_code_ != 0 && flip_code_ != 1 && flip_code_ != -1) {
    RCLCPP_WARN(this->get_logger(), 
                "Invalid flip_code: %d. Using -1 (both axes). Valid values: 0 (vertical), 1 (horizontal), -1 (both)", 
                flip_code_);
    flip_code_ = -1;
  }

  // Create subscription
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, 10,
    std::bind(&ImageFlipNode::image_callback, this, std::placeholders::_1));

  // Create publisher
  image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_topic_, 10);

  if (flip_enabled_) {
    const char* flip_mode = (flip_code_ == 0) ? "vertical" : 
                           (flip_code_ == 1) ? "horizontal" : "both axes";
    RCLCPP_INFO(this->get_logger(), 
                "Image flip node initialized: %s -> %s (flip: %s)", 
                input_topic_.c_str(), output_topic_.c_str(), flip_mode);
  } else {
    RCLCPP_INFO(this->get_logger(), 
                "Image flip node initialized: %s -> %s (flip: disabled)", 
                input_topic_.c_str(), output_topic_.c_str());
  }
}

void ImageFlipNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Convert ROS image to OpenCV, preserving original encoding
    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Try to use the original encoding first
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
      // If that fails, try common encodings
      try {
        if (msg->encoding == "rgb8" || msg->encoding == "RGB8") {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        } else if (msg->encoding == "bgr8" || msg->encoding == "BGR8") {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } else {
          // Default to BGR8 for OpenCV compatibility
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          RCLCPP_WARN_ONCE(this->get_logger(), 
                          "Unsupported encoding '%s', converting to BGR8. Image may have incorrect colors.", 
                          msg->encoding.c_str());
        }
      } catch (cv_bridge::Exception& e2) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e2.what());
        return;
      }
    }

    // Flip image if enabled
    if (flip_enabled_) {
      cv::flip(cv_ptr->image, cv_ptr->image, flip_code_);
    }

    // Convert back to ROS image and publish, preserving original encoding
    cv_ptr->encoding = msg->encoding;
    sensor_msgs::msg::Image::SharedPtr output_msg = cv_ptr->toImageMsg();
    output_msg->header = msg->header;  // Preserve header
    image_publisher_->publish(*output_msg);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
  }
}

}  // namespace sensors

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensors::ImageFlipNode>());
  rclcpp::shutdown();
  return 0;
}

