#include "imitation_learning/imitation_learning_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>

using namespace imitation_learning;

ImitationLearningNode::ImitationLearningNode(const rclcpp::NodeOptions& options)
  : Node("imitation_learning_node", options),
    latest_linear_x_(0.0f),
    latest_angular_z_(0.0f),
    has_prediction_(false)
{
  // Initialize parameters
  this->declare_parameter("model_path", "/home/rcr/repos/common_platform/models/imitation_learning.hef");
  this->declare_parameter("input_width", 224);
  this->declare_parameter("input_height", 224);
  this->declare_parameter("sequence_length", 10);
  this->declare_parameter("max_linear_velocity", 0.5);
  this->declare_parameter("max_angular_velocity", 1.5);
  this->declare_parameter("publish_rate", 30.0);
  this->declare_parameter("hailort_log_path", "/tmp");

  model_path_ = this->get_parameter("model_path").as_string();
  input_width_ = this->get_parameter("input_width").as_int();
  input_height_ = this->get_parameter("input_height").as_int();
  sequence_length_ = this->get_parameter("sequence_length").as_int();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_float();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_float();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  
  std::string log_path = this->get_parameter("hailort_log_path").as_string();
  RCLCPP_INFO(this->get_logger(), "HailoRT log directory: %s (hailort.log will be created here)", log_path.c_str());

  RCLCPP_INFO(this->get_logger(), "Initializing Imitation Learning Node");
  RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Input dimensions: %dx%d", input_width_, input_height_);
  RCLCPP_INFO(this->get_logger(), "Sequence length: %d", sequence_length_);
  RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate_);

  // Initialize HailoRT
#ifdef HAVE_HAILORT
  try {
    initialize_hailo();
    RCLCPP_INFO(this->get_logger(), "HailoRT initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize HailoRT: %s", e.what());
    return;
  }
#else
  RCLCPP_WARN(this->get_logger(), "HailoRT not available - running in simulation mode");
#endif

  // Create subscription to camera/image_raw topic
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", 10,
    std::bind(&ImitationLearningNode::image_callback, this, std::placeholders::_1));

  // Create publisher for cmd_vel
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);

  // Create timer to publish cmd_vel at specified rate
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
  publish_timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&ImitationLearningNode::publish_cmd_vel_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Imitation learning node initialized");
}

ImitationLearningNode::~ImitationLearningNode()
{
  // Cleanup HailoRT resources
#ifdef HAVE_HAILORT
  RCLCPP_INFO(this->get_logger(), "Shutting down HailoRT resources");
  
  output_vstreams_.clear();
  input_vstreams_.clear();
  activated_network_group_.reset();
  configured_network_group_.reset();
  device_.reset();
  
  RCLCPP_INFO(this->get_logger(), "HailoRT resources cleaned up");
#endif
}

void ImitationLearningNode::initialize_hailo()
{
#ifdef HAVE_HAILORT
  try {
    RCLCPP_INFO(this->get_logger(), "Initializing HailoRT with model: %s", model_path_.c_str());
    
    // 1. Load the HEF model
    auto hef_expected = hailort::Hef::create(model_path_);
    if (!hef_expected) {
      throw std::runtime_error("Failed to load HEF model from: " + model_path_);
    }
    auto hef = hef_expected.release();
    RCLCPP_INFO(this->get_logger(), "HEF model loaded successfully");
    
    // 2. Create Hailo device (try PCIe first, then default)
    auto device_expected = hailort::Device::create_pcie();
    if (!device_expected) {
      RCLCPP_WARN(this->get_logger(), "PCIe device not found, trying default device");
      auto device_expected2 = hailort::Device::create();
      if (!device_expected2) {
        throw std::runtime_error("Failed to create Hailo device");
      }
      device_ = device_expected2.release();
    } else {
      device_ = device_expected.release();
    }
    RCLCPP_INFO(this->get_logger(), "Hailo device created successfully");
    
    // 3. Create configure parameters
    auto configure_params_expected = device_->create_configure_params(hef);
    if (!configure_params_expected) {
      throw std::runtime_error("Failed to create configure parameters");
    }
    auto configure_params = configure_params_expected.release();
    
    // 4. Configure the device with the HEF
    auto network_groups_expected = device_->configure(hef, configure_params);
    if (!network_groups_expected) {
      throw std::runtime_error("Failed to configure device");
    }
    auto network_groups = network_groups_expected.release();
    
    if (network_groups.empty()) {
      throw std::runtime_error("No network groups configured");
    }
    
    // Use the first network group
    configured_network_group_ = network_groups[0];
    RCLCPP_INFO(this->get_logger(), "Network group configured successfully");
    
    // 5. Create input and output vstreams
    auto input_vstream_params_expected = configured_network_group_->make_input_vstream_params(
      false, HAILO_FORMAT_TYPE_AUTO, 1000, 10);
    if (!input_vstream_params_expected) {
      throw std::runtime_error("Failed to make input vstream params");
    }
    auto input_vstream_params = input_vstream_params_expected.release();
    
    auto output_vstream_params_expected = configured_network_group_->make_output_vstream_params(
      false, HAILO_FORMAT_TYPE_AUTO, 1000, 10);
    if (!output_vstream_params_expected) {
      throw std::runtime_error("Failed to make output vstream params");
    }
    auto output_vstream_params = output_vstream_params_expected.release();
    
    // Create input vstreams
    auto input_vstreams_expected = hailort::VStreamsBuilder::create_input_vstreams(
      *configured_network_group_, input_vstream_params);
    if (!input_vstreams_expected) {
      throw std::runtime_error("Failed to create input vstreams");
    }
    auto input_vstreams_vec = input_vstreams_expected.release();
    for (auto& vstream : input_vstreams_vec) {
      input_vstreams_.push_back(std::make_shared<hailort::InputVStream>(std::move(vstream)));
    }
    
    // Create output vstreams
    auto output_vstreams_expected = hailort::VStreamsBuilder::create_output_vstreams(
      *configured_network_group_, output_vstream_params);
    if (!output_vstreams_expected) {
      throw std::runtime_error("Failed to create output vstreams");
    }
    auto output_vstreams_vec = output_vstreams_expected.release();
    for (auto& vstream : output_vstreams_vec) {
      output_vstreams_.push_back(std::make_shared<hailort::OutputVStream>(std::move(vstream)));
    }
    
    // 6. Activate the network group
    auto activated_network_group_expected = configured_network_group_->activate();
    if (!activated_network_group_expected) {
      throw std::runtime_error("Failed to activate network group");
    }
    activated_network_group_ = activated_network_group_expected.release();
    RCLCPP_INFO(this->get_logger(), "Network group activated successfully");
    
    RCLCPP_INFO(this->get_logger(), "HailoRT initialized successfully with %zu input and %zu output vstreams",
                input_vstreams_.size(), output_vstreams_.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "HailoRT initialization failed: %s", e.what());
    throw;
  }
#else
  throw std::runtime_error("HailoRT not available");
#endif
}

void ImitationLearningNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Convert ROS image to OpenCV Mat
    cv::Mat image;
    if (msg->encoding == "rgb8") {
      image = cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
    } else if (msg->encoding == "bgr8") {
      cv::Mat bgr_image = cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
      cv::cvtColor(bgr_image, image, cv::COLOR_BGR2RGB);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    // Preprocess and add to buffer
    cv::Mat processed_image;
    preprocess_image(image, processed_image);

    // Add to buffer
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      image_buffer_.push_back(processed_image);
      
      // Keep only the last sequence_length images
      if (image_buffer_.size() > static_cast<size_t>(sequence_length_)) {
        image_buffer_.pop_front();
      }
    }

    // Run inference when we have enough images
    if (image_buffer_.size() >= static_cast<size_t>(sequence_length_)) {
      // For now, use the most recent image (single frame inference)
      // TODO: Implement proper sequence processing when model supports it
      cv::Mat inference_image;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        inference_image = image_buffer_.back().clone();
      }

      auto predictions = run_inference(inference_image);
      
      if (predictions.size() >= 2) {
        std::lock_guard<std::mutex> lock(prediction_mutex_);
        latest_linear_x_ = predictions[0];
        latest_angular_z_ = predictions[1];
        has_prediction_ = true;
        
        // Clamp to max velocities
        latest_linear_x_ = std::clamp(latest_linear_x_, -max_linear_velocity_, max_linear_velocity_);
        latest_angular_z_ = std::clamp(latest_angular_z_, -max_angular_velocity_, max_angular_velocity_);
        
        RCLCPP_DEBUG(this->get_logger(), "Prediction: linear_x=%.3f, angular_z=%.3f",
                     latest_linear_x_, latest_angular_z_);
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
  }
}

void ImitationLearningNode::preprocess_image(const cv::Mat& image, cv::Mat& output)
{
  // Resize to model input size
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(input_width_, input_height_));
  
  // Normalize to [0, 1] and convert to RGB if needed
  resized.convertTo(output, CV_32FC3, 1.0 / 255.0);
}

std::vector<float> ImitationLearningNode::run_inference(const cv::Mat& image)
{
  std::vector<float> predictions;

  try {
#ifdef HAVE_HAILORT
    if (!input_vstreams_.empty() && !output_vstreams_.empty()) {
      // Preprocess image for HailoRT
      cv::Mat resized_image;
      cv::resize(image, resized_image, cv::Size(input_width_, input_height_));
      
      // Convert to RGB if needed (HailoRT expects RGB)
      cv::Mat rgb_image;
      if (image.channels() == 3) {
        cv::cvtColor(resized_image, rgb_image, cv::COLOR_BGR2RGB);
      } else {
        rgb_image = resized_image;
      }
      
      // Normalize to [0, 1] range and convert to float
      cv::Mat float_image;
      rgb_image.convertTo(float_image, CV_32FC3, 1.0 / 255.0);
      
      // Send to input vstream
      auto& input_vstream = input_vstreams_[0];
      size_t input_frame_size = input_vstream->get_frame_size();
      size_t image_size = float_image.total() * float_image.elemSize();
      
      if (input_frame_size != image_size) {
        RCLCPP_ERROR(this->get_logger(), "Input size mismatch: expected %zu bytes, got %zu bytes",
                     input_frame_size, image_size);
        return predictions;
      }
      
      auto status = input_vstream->write(hailort::MemoryView(float_image.data, input_frame_size));
      if (status != HAILO_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to input vstream");
        return predictions;
      }
      
      // Read results from output vstream
      for (auto& output_vstream : output_vstreams_) {
        auto output_info = output_vstream->get_info();
        size_t output_size = output_vstream->get_frame_size();
        std::vector<uint8_t> output_buffer(output_size);
        
        status = output_vstream->read(hailort::MemoryView(output_buffer.data(), output_buffer.size()));
        if (status != HAILO_SUCCESS) {
          RCLCPP_ERROR(this->get_logger(), "Failed to read from output vstream");
          continue;
        }
        
        // Parse output (assuming 2 float values: linear_x, angular_z)
        const float* output_float = reinterpret_cast<const float*>(output_buffer.data());
        int num_outputs = output_size / sizeof(float);
        
        for (int i = 0; i < num_outputs && i < 2; i++) {
          predictions.push_back(output_float[i]);
        }
      }
      
      RCLCPP_DEBUG(this->get_logger(), "HailoRT inference completed with %zu predictions", predictions.size());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "HailoRT vstreams not available");
    }
#else
    // Simulation mode - create dummy predictions for testing
    RCLCPP_DEBUG(this->get_logger(), "Running simulation inference");
    predictions.push_back(0.1f);  // linear_x
    predictions.push_back(0.0f);  // angular_z
#endif

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
  }

  return predictions;
}

void ImitationLearningNode::publish_cmd_vel_timer_callback()
{
  geometry_msgs::msg::Twist twist_msg;
  
  {
    std::lock_guard<std::mutex> lock(prediction_mutex_);
    if (has_prediction_) {
      twist_msg.linear.x = latest_linear_x_;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = latest_angular_z_;
    } else {
      // No prediction yet, publish zero velocities
      twist_msg.linear.x = 0.0;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = 0.0;
    }
  }
  
  cmd_vel_publisher_->publish(twist_msg);
  
  RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear.x=%.3f, angular.z=%.3f",
               twist_msg.linear.x, twist_msg.angular.z);
}

int main(int argc, char* argv[])
{
  // Set HailoRT log directory BEFORE any HailoRT initialization
  setenv("HAILORT_LOGGER_PATH", "/tmp", 1);
  
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ImitationLearningNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

