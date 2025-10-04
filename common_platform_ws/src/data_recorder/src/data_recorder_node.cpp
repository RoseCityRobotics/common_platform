#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <signal.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>
#include <thread>
#include <atomic>

// Forward declarations
class DataRecorder;
void signalHandler(int signal);

// Global variables for signal handling
std::atomic<bool> g_shutdown_requested{false};
DataRecorder* g_data_recorder = nullptr;

class DataRecorder : public rclcpp::Node
{
public:
  DataRecorder() : Node("data_recorder"), ramdisk_mounted_(false)
  {
    // Set global pointer for signal handler
    g_data_recorder = this;
    
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    // Declare parameters
    this->declare_parameter("output_dir", "/tmp/teleop_data");
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("odom_topic", "/odom");
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
    odom_topic_ = this->get_parameter("odom_topic").as_string();
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
    data_file_ << "timestamp,image_file,linear_vel,angular_vel,position_x,position_y,orientation_z\n";
    
    // Create subscribers
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10,
      std::bind(&DataRecorder::cameraCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&DataRecorder::odomCallback, this, std::placeholders::_1));
    
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
    last_odom_.twist.twist.linear.x = 0.0;
    last_odom_.twist.twist.angular.z = 0.0;
    last_odom_.pose.pose.position.x = 0.0;
    last_odom_.pose.pose.position.y = 0.0;
    last_odom_.pose.pose.orientation.z = 0.0;
    last_image_ = nullptr;
    
    // Mount RAM disk if output directory is in /mnt/recording_ramdisk
    if (output_dir_.find("/mnt/recording_ramdisk") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "Mounting RAM disk for fast recording...");
      if (!mountRamdisk("/mnt/recording_ramdisk", "2G")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to mount RAM disk, continuing with regular storage");
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Data recorder initialized");
    RCLCPP_INFO(this->get_logger(), "Output directory: %s", session_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera type: %s", camera_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera ID: %s", camera_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Odometry topic: %s", odom_topic_.c_str());
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
    
    // Run cleanup on destruction
    shutdown();
  }

  // RAM disk management methods
  bool mountRamdisk(const std::string& ramdisk_path, const std::string& size = "2G")
  {
    // Create mount point
    std::filesystem::create_directories(ramdisk_path);
    
    // Check if already mounted
    struct stat statbuf;
    if (stat(ramdisk_path.c_str(), &statbuf) == 0) {
      // Check if it's a tmpfs mount
      std::ifstream mounts("/proc/mounts");
      std::string line;
      while (std::getline(mounts, line)) {
        if (line.find(ramdisk_path) != std::string::npos && line.find("tmpfs") != std::string::npos) {
          RCLCPP_INFO(this->get_logger(), "RAM disk already mounted at %s", ramdisk_path.c_str());
          ramdisk_mounted_ = true;
          return true;
        }
      }
    }
    
    // Try to mount using system call first (requires sudoers config)
    std::string mount_cmd = "sudo mount -t tmpfs -o size=" + size + " tmpfs " + ramdisk_path;
    int result = system(mount_cmd.c_str());
    
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "RAM disk mounted successfully at %s", ramdisk_path.c_str());
      ramdisk_mounted_ = true;
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to mount RAM disk with sudo, trying alternative approach...");
      
      // Alternative: Use regular user mount (if available)
      std::string user_mount_cmd = "mount -t tmpfs -o size=" + size + " tmpfs " + ramdisk_path;
      result = system(user_mount_cmd.c_str());
      
      if (result == 0) {
        RCLCPP_INFO(this->get_logger(), "RAM disk mounted successfully at %s", ramdisk_path.c_str());
        ramdisk_mounted_ = true;
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to mount RAM disk at %s - continuing without RAM disk", ramdisk_path.c_str());
        return false;
      }
    }
  }
  
  bool unmountRamdisk(const std::string& ramdisk_path)
  {
    if (!ramdisk_mounted_) {
      return true;
    }
    
    // Try sudo first
    std::string unmount_cmd = "sudo umount " + ramdisk_path;
    int result = system(unmount_cmd.c_str());
    
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "RAM disk unmounted successfully");
      ramdisk_mounted_ = false;
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to unmount with sudo, trying regular umount...");
      
      // Try regular umount
      std::string user_unmount_cmd = "umount " + ramdisk_path;
      result = system(user_unmount_cmd.c_str());
      
      if (result == 0) {
        RCLCPP_INFO(this->get_logger(), "RAM disk unmounted successfully");
        ramdisk_mounted_ = false;
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to unmount RAM disk");
        return false;
      }
    }
  }
  
  void copyWithProgress(const std::string& src, const std::string& dst)
  {
    if (!std::filesystem::exists(src)) {
      RCLCPP_WARN(this->get_logger(), "Source path does not exist: %s", src.c_str());
      return;
    }
    
    // Create destination directory
    std::filesystem::create_directories(dst);
    
    // Get list of files to copy
    std::vector<std::filesystem::path> files_to_copy;
    size_t total_size = 0;
    
    for (const auto& entry : std::filesystem::recursive_directory_iterator(src)) {
      if (entry.is_regular_file()) {
        files_to_copy.push_back(entry.path());
        total_size += std::filesystem::file_size(entry.path());
      }
    }
    
    if (files_to_copy.empty()) {
      RCLCPP_WARN(this->get_logger(), "No files to copy");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Copying %zu files (%.1f MB)...", 
                files_to_copy.size(), total_size / (1024.0 * 1024.0));
    
    // Copy files with progress
    size_t copied_size = 0;
    size_t copied_files = 0;
    
    for (const auto& src_file : files_to_copy) {
      // Calculate relative path
      auto rel_path = std::filesystem::relative(src_file, src);
      auto dst_file = std::filesystem::path(dst) / rel_path;
      
      // Create destination directory
      std::filesystem::create_directories(dst_file.parent_path());
      
      // Copy file
      std::filesystem::copy_file(src_file, dst_file, std::filesystem::copy_options::overwrite_existing);
      
      // Update progress
      copied_size += std::filesystem::file_size(src_file);
      copied_files++;
      
      // Show progress
      double progress = static_cast<double>(copied_size) / total_size;
      int bar_length = 50;
      int filled_length = static_cast<int>(bar_length * progress);
      std::string bar = std::string(filled_length, '#') + std::string(bar_length - filled_length, '-');
      
      printf("\r[%s] %.1f%% (%zu/%zu files) %.1fMB", 
             bar.c_str(), progress * 100.0, copied_files, files_to_copy.size(), 
             copied_size / (1024.0 * 1024.0));
      fflush(stdout);
    }
    
    printf("\n");
    RCLCPP_INFO(this->get_logger(), "Copy completed successfully");
  }
  
  void shutdown()
  {
    static bool shutdown_called = false;
    if (shutdown_called) {
      return; // Prevent double cleanup
    }
    shutdown_called = true;
    
    RCLCPP_INFO(this->get_logger(), "=== Shutting down gracefully ===");
    
    // Stop recording
    if (recording_) {
      recording_ = false;
      RCLCPP_INFO(this->get_logger(), "Recording stopped");
    }
    
    // Copy data from RAM disk to SD card
    if (ramdisk_mounted_) {
      RCLCPP_INFO(this->get_logger(), "Backing up data to SD card...");
      
      // Find the session directory in the current session_dir_
      if (std::filesystem::exists(session_dir_)) {
        // Create backup directory with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        
        std::string backup_path = "/home/rcr/teleop_data/" + 
                                 std::filesystem::path(session_dir_).filename().string() + 
                                 "_backup_" + ss.str();
        
        RCLCPP_INFO(this->get_logger(), "Backing up session: %s", session_dir_.c_str());
        copyWithProgress(session_dir_, backup_path);
      } else {
        RCLCPP_WARN(this->get_logger(), "Session directory not found: %s", session_dir_.c_str());
      }
    }
    
    // Unmount RAM disk
    if (ramdisk_mounted_) {
      unmountRamdisk("/mnt/recording_ramdisk");
    }
    
    RCLCPP_INFO(this->get_logger(), "Shutdown completed");
  }

private:
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_image_ = msg;
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_odom_ = *msg;
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
      
      // Extract odometry data
      double linear_vel = last_odom_.twist.twist.linear.x;
      double angular_vel = last_odom_.twist.twist.angular.z;
      double position_x = last_odom_.pose.pose.position.x;
      double position_y = last_odom_.pose.pose.position.y;
      double orientation_z = last_odom_.pose.pose.orientation.z;
      
      // Get timestamp
      auto now = std::chrono::system_clock::now();
      auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
      
      // Write to data log
      data_file_ << timestamp << ","
                 << image_filename << ","
                 << linear_vel << ","
                 << angular_vel << ","
                 << position_x << ","
                 << position_y << ","
                 << orientation_z << "\n";
      
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
  std::string odom_topic_;
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
  bool ramdisk_mounted_;
  
  // Data storage
  sensor_msgs::msg::Image::SharedPtr last_image_;
  nav_msgs::msg::Odometry last_odom_;
  std::mutex data_mutex_;
  
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_control_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Signal handler (defined after class)
void signalHandler(int signal)
{
  RCLCPP_INFO(rclcpp::get_logger("data_recorder"), "Received signal %d, shutting down gracefully...", signal);
  g_shutdown_requested = true;
  if (g_data_recorder) {
    g_data_recorder->shutdown();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<DataRecorder>();
  
  // Spin until shutdown is requested
  while (rclcpp::ok() && !g_shutdown_requested) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  rclcpp::shutdown();
  return 0;
}
