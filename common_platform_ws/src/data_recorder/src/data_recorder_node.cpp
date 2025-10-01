#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

class DataRecorder : public rclcpp::Node
{
public:
  DataRecorder() : Node("data_recorder")
  {
    // Declare parameters
    this->declare_parameter("output_dir", "/tmp/teleop_data");
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("record_rate", 30.0);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("auto_start", false);
    this->declare_parameter("camera_type", "libcamera");
    this->declare_parameter("camera_id", "0");
    this->declare_parameter("pixel_format", "");
    
    // Get parameters
    output_dir_ = this->get_parameter("output_dir").as_string();
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    record_rate_ = this->get_parameter("record_rate").as_double();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    auto_start_ = this->get_parameter("auto_start").as_bool();
    camera_type_ = this->get_parameter("camera_type").as_string();
    camera_id_ = this->get_parameter("camera_id").as_string();
    pixel_format_ = this->get_parameter("pixel_format").as_string();
    
    // Create output directory with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    session_dir_ = output_dir_ + "/session_" + ss.str();
    std::filesystem::create_directories(session_dir_);
    
    // Create subdirectories
    std::filesystem::create_directories(session_dir_ + "/images");
    std::filesystem::create_directories(session_dir_ + "/metadata");
    
    // Initialize data file
    data_file_.open(session_dir_ + "/metadata/data_log.csv");
    data_file_ << "timestamp,image_file,linear_vel,angular_vel,left_wheel_vel,right_wheel_vel\n";
    
    // Create subscribers
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10,
      std::bind(&DataRecorder::cameraCallback, this, std::placeholders::_1));
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&DataRecorder::cmdVelCallback, this, std::placeholders::_1));
    
    // Create recording control subscriber (use relative topic name for namespace support)
    record_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "data_recorder/start_stop", 10,
      std::bind(&DataRecorder::recordControlCallback, this, std::placeholders::_1));
    
    // Create status publisher (use relative topic name for namespace support)
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("data_recorder/status", 10);
    
    // Create timer for synchronized recording
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / record_rate_)),
      std::bind(&DataRecorder::recordData, this));
    
    // Initialize state
    recording_ = auto_start_;
    frame_count_ = 0;
    last_cmd_vel_.linear.x = 0.0;
    last_cmd_vel_.angular.z = 0.0;
    last_image_ = nullptr;
    
    RCLCPP_INFO(this->get_logger(), "Data recorder initialized");
    RCLCPP_INFO(this->get_logger(), "Output directory: %s", session_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera type: %s", camera_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera ID: %s", camera_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Command velocity topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Recording rate: %.1f Hz", record_rate_);
    RCLCPP_INFO(this->get_logger(), "Recording: %s", recording_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "Publish to data_recorder/start_stop to control recording");
  }
  
  ~DataRecorder()
  {
    if (data_file_.is_open()) {
      data_file_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Data recording session ended. Total frames: %lu", frame_count_);
  }

private:
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_image_ = msg;
  }
  
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_cmd_vel_ = *msg;
  }
  
  void recordControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    recording_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Recording %s", recording_ ? "STARTED" : "STOPPED");
    
    // Publish status update
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = recording_;
    status_pub_->publish(status_msg);
  }
  
  void recordData()
  {
    if (!recording_) {
      return;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Check if we have both image and command data
    if (!last_image_) {
      return;
    }
    
    try {
      // Convert ROS image to OpenCV
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;
      
      // Resize image if needed
      if (image.cols != image_width_ || image.rows != image_height_) {
        cv::resize(image, image, cv::Size(image_width_, image_height_));
      }
      
      // Generate filename with frame count
      std::stringstream filename_ss;
      filename_ss << std::setfill('0') << std::setw(8) << frame_count_ << ".jpg";
      std::string image_filename = filename_ss.str();
      std::string image_path = session_dir_ + "/images/" + image_filename;
      
      // Save image
      cv::imwrite(image_path, image);
      
      // Calculate wheel velocities (assuming differential drive)
      // These values should match your robot's configuration
      const double wheel_separation = 0.297;  // meters (from your config)
      const double wheel_radius = 0.033;      // meters (from your config)
      
      double left_wheel_vel = (last_cmd_vel_.linear.x - last_cmd_vel_.angular.z * wheel_separation / 2.0) / wheel_radius;
      double right_wheel_vel = (last_cmd_vel_.linear.x + last_cmd_vel_.angular.z * wheel_separation / 2.0) / wheel_radius;
      
      // Get timestamp
      auto now = std::chrono::system_clock::now();
      auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
      
      // Write to data log
      data_file_ << timestamp << ","
                 << image_filename << ","
                 << last_cmd_vel_.linear.x << ","
                 << last_cmd_vel_.angular.z << ","
                 << left_wheel_vel << ","
                 << right_wheel_vel << "\n";
      
      frame_count_++;
      
      // Log progress every 100 frames
      if (frame_count_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Recorded %lu frames", frame_count_);
      }
      
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
  
  // Parameters
  std::string output_dir_;
  std::string camera_topic_;
  std::string cmd_vel_topic_;
  double record_rate_;
  int image_width_;
  int image_height_;
  bool auto_start_;
  std::string camera_type_;
  std::string camera_id_;
  std::string pixel_format_;
  
  // State variables
  std::string session_dir_;
  std::ofstream data_file_;
  bool recording_;
  size_t frame_count_;
  
  // Data storage
  sensor_msgs::msg::Image::SharedPtr last_image_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  std::mutex data_mutex_;
  
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_control_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataRecorder>());
  rclcpp::shutdown();
  return 0;
}
