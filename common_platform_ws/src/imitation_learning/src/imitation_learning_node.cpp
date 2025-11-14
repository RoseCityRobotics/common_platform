#include "imitation_learning/imitation_learning_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace imitation_learning;

ImitationLearningNode::ImitationLearningNode(const rclcpp::NodeOptions& options)
  : Node("imitation_learning_node", options),
#ifdef HAVE_ONNXRUNTIME
    ort_env_(ORT_LOGGING_LEVEL_WARNING, "ImitationLearningNode"),
#endif
    latest_linear_x_(0.0f),
    latest_angular_z_(0.0f),
    has_prediction_(false),
    inference_in_progress_(false)
{
  // Initialize parameters
  this->declare_parameter("model_path", "/home/rcr/repos/common_platform/models/imitation_learning.onnx");
  this->declare_parameter("input_width", 224);
  this->declare_parameter("input_height", 224);
  this->declare_parameter("sequence_length", 10);
  this->declare_parameter("max_linear_velocity", 0.5);
  this->declare_parameter("max_angular_velocity", 1.5);
  this->declare_parameter("publish_rate", 30.0);
  this->declare_parameter("stats_report_interval", 10.0);  // Report stats every 10 seconds
  this->declare_parameter("max_inference_time_ms", 33.0);  // Max inference time for 30 Hz (33.3 ms per frame)

  model_path_ = this->get_parameter("model_path").as_string();
  input_width_ = this->get_parameter("input_width").as_int();
  input_height_ = this->get_parameter("input_height").as_int();
  sequence_length_ = this->get_parameter("sequence_length").as_int();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_float();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_float();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  double stats_interval = this->get_parameter("stats_report_interval").as_double();

  RCLCPP_INFO(this->get_logger(), "Initializing Imitation Learning Node");
  RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Input dimensions: %dx%d", input_width_, input_height_);
  RCLCPP_INFO(this->get_logger(), "Sequence length: %d", sequence_length_);
  RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate_);

  // Initialize ONNX Runtime
#ifdef HAVE_ONNXRUNTIME
  try {
    initialize_onnx();
    RCLCPP_INFO(this->get_logger(), "ONNX Runtime initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize ONNX Runtime: %s", e.what());
    return;
  }
#else
  RCLCPP_ERROR(this->get_logger(), "ONNX Runtime not available - cannot run inference");
  return;
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

  // Create timer for stats reporting
  auto stats_period = std::chrono::milliseconds(static_cast<int>(stats_interval * 1000.0));
  stats_timer_ = this->create_wall_timer(
    stats_period,
    std::bind(&ImitationLearningNode::stats_report_timer_callback, this));

  // Initialize stats
  stats_.last_stats_report = std::chrono::steady_clock::now();
  last_image_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "Imitation learning node initialized");
}

ImitationLearningNode::~ImitationLearningNode()
{
#ifdef HAVE_ONNXRUNTIME
  RCLCPP_INFO(this->get_logger(), "Shutting down ONNX Runtime resources");
  session_.reset();
  RCLCPP_INFO(this->get_logger(), "ONNX Runtime resources cleaned up");
#endif
}

void ImitationLearningNode::initialize_onnx()
{
#ifdef HAVE_ONNXRUNTIME
  try {
    RCLCPP_INFO(this->get_logger(), "Initializing ONNX Runtime with model: %s", model_path_.c_str());
    
    // Check if model file exists
    std::ifstream model_file(model_path_);
    if (!model_file.good()) {
      throw std::runtime_error("Model file not found: " + model_path_);
    }
    model_file.close();

    // Configure session options
    session_options_.SetIntraOpNumThreads(4);  // Use 4 threads for intra-op parallelism
    session_options_.SetInterOpNumThreads(2);  // Use 2 threads for inter-op parallelism
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    
    // Create session
    session_ = std::make_unique<Ort::Session>(ort_env_, model_path_.c_str(), session_options_);
    
    // Get input/output names and shapes
    size_t num_input_nodes = session_->GetInputCount();
    size_t num_output_nodes = session_->GetOutputCount();
    
    RCLCPP_INFO(this->get_logger(), "Model has %zu input(s) and %zu output(s)", num_input_nodes, num_output_nodes);
    
    // Storage for input/output names (need to keep them alive)
    std::vector<std::string> input_name_strings;
    std::vector<std::string> output_name_strings;
    
    // Get input information
    for (size_t i = 0; i < num_input_nodes; i++) {
      auto input_name = session_->GetInputNameAllocated(i, allocator_);
      input_name_strings.push_back(std::string(input_name.get()));
      input_names_.push_back(input_name_strings.back().c_str());
      
      auto input_type_info = session_->GetInputTypeInfo(i);
      auto tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
      auto shape = tensor_info.GetShape();
      input_shapes_.push_back(shape);
      
      std::stringstream shape_ss;
      for (size_t j = 0; j < shape.size(); j++) {
        if (j > 0) shape_ss << ",";
        shape_ss << shape[j];
      }
      
      RCLCPP_INFO(this->get_logger(), "Input %zu: name='%s', shape=[%s]", 
                  i, input_names_[i], shape_ss.str().c_str());
    }
    
    // Get output information
    for (size_t i = 0; i < num_output_nodes; i++) {
      auto output_name = session_->GetOutputNameAllocated(i, allocator_);
      output_name_strings.push_back(std::string(output_name.get()));
      output_names_.push_back(output_name_strings.back().c_str());
      
      auto output_type_info = session_->GetOutputTypeInfo(i);
      auto tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
      auto shape = tensor_info.GetShape();
      output_shapes_.push_back(shape);
      
      std::stringstream shape_ss;
      for (size_t j = 0; j < shape.size(); j++) {
        if (j > 0) shape_ss << ",";
        shape_ss << shape[j];
      }
      
      RCLCPP_INFO(this->get_logger(), "Output %zu: name='%s', shape=[%s]", 
                  i, output_names_[i], shape_ss.str().c_str());
    }
    
    // Store name strings as member variables to keep them alive
    input_name_strings_ = std::move(input_name_strings);
    output_name_strings_ = std::move(output_name_strings);
    
    RCLCPP_INFO(this->get_logger(), "ONNX Runtime initialized successfully");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "ONNX Runtime initialization failed: %s", e.what());
    throw;
  }
#else
  throw std::runtime_error("ONNX Runtime not available");
#endif
}

void ImitationLearningNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto current_time = std::chrono::steady_clock::now();
  auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
    current_time - last_image_time_).count();
  last_image_time_ = current_time;

  // Check if we're keeping up with camera rate
  double expected_interval_ms = 1000.0 / publish_rate_;
  if (time_since_last > expected_interval_ms * 1.5) {
    stats_.dropped_frames++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Dropped frame detected: %ld ms since last image (expected ~%.1f ms)",
                         time_since_last, expected_interval_ms);
  }

  try {
    // Skip if inference is already in progress (to avoid queueing)
    {
      std::lock_guard<std::mutex> lock(inference_mutex_);
      if (inference_in_progress_) {
        stats_.dropped_frames++;
        return;
      }
      inference_in_progress_ = true;
    }

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
      {
        std::lock_guard<std::mutex> lock(inference_mutex_);
        inference_in_progress_ = false;
      }
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

    {
      std::lock_guard<std::mutex> lock(inference_mutex_);
      inference_in_progress_ = false;
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
    {
      std::lock_guard<std::mutex> lock(inference_mutex_);
      inference_in_progress_ = false;
    }
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
  auto inference_start = std::chrono::steady_clock::now();

  try {
#ifdef HAVE_ONNXRUNTIME
    if (!session_) {
      RCLCPP_ERROR(this->get_logger(), "ONNX session not initialized");
      return predictions;
    }

    // Prepare input tensor
    // Expected input shape: (batch, seq_len, channels, height, width) = (1, 10, 3, 224, 224)
    // For now, we'll use single frame: (1, 1, 3, 224, 224) and repeat it sequence_length times
    // TODO: Implement proper sequence processing
    
    // Resize and normalize image
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(input_width_, input_height_));
    
    // Convert to RGB if needed
    cv::Mat rgb_image;
    if (image.channels() == 3) {
      cv::cvtColor(resized_image, rgb_image, cv::COLOR_BGR2RGB);
    } else {
      rgb_image = resized_image;
    }
    
    // Normalize to [0, 1] range
    cv::Mat float_image;
    rgb_image.convertTo(float_image, CV_32FC3, 1.0 / 255.0);
    
    // Create sequence by repeating the image
    // Input shape: (1, sequence_length, 3, height, width)
    size_t input_size = 1 * sequence_length_ * 3 * input_height_ * input_width_;
    std::vector<float> input_data(input_size);
    
    // Copy image data sequence_length times
    float* input_ptr = input_data.data();
    for (int i = 0; i < sequence_length_; i++) {
      std::memcpy(input_ptr, float_image.data, 3 * input_height_ * input_width_ * sizeof(float));
      input_ptr += 3 * input_height_ * input_width_;
    }
    
    // Create input tensor
    std::vector<int64_t> input_shape = {1, sequence_length_, 3, input_height_, input_width_};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, input_data.data(), input_size, input_shape.data(), input_shape.size());
    
    // Run inference
    auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                        input_names_.data(), &input_tensor, 1,
                                        output_names_.data(), output_names_.size());
    
    // Extract output
    if (!output_tensors.empty() && output_tensors.front().IsTensor()) {
      float* output_data = output_tensors.front().GetTensorMutableData<float>();
      auto output_shape = output_tensors.front().GetTensorTypeAndShapeInfo().GetShape();
      
      // Expected output shape: (batch, chunk_size, action_dim) = (1, 15, 2)
      // Extract first action: [0, 0, :] = (linear_x, angular_z)
      size_t chunk_size = output_shape.size() > 1 ? output_shape[1] : 1;
      size_t action_dim = output_shape.size() > 2 ? output_shape[2] : output_shape.back();
      
      if (action_dim >= 2) {
        predictions.push_back(output_data[0]);  // linear_x
        predictions.push_back(output_data[1]);  // angular_z
      } else {
        RCLCPP_WARN(this->get_logger(), "Unexpected output shape or insufficient dimensions");
      }
    }
    
    // Measure inference time
    auto inference_end = std::chrono::steady_clock::now();
    auto inference_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      inference_end - inference_start).count() / 1000.0;
    
    // Update statistics
    stats_.total_inferences++;
    stats_.total_inference_time_ms += inference_time_ms;
    
    double current_min = stats_.min_inference_time_ms.load();
    while (inference_time_ms < current_min && 
           !stats_.min_inference_time_ms.compare_exchange_weak(current_min, inference_time_ms)) {
      current_min = stats_.min_inference_time_ms.load();
    }
    
    double current_max = stats_.max_inference_time_ms.load();
    while (inference_time_ms > current_max && 
           !stats_.max_inference_time_ms.compare_exchange_weak(current_max, inference_time_ms)) {
      current_max = stats_.max_inference_time_ms.load();
    }
    
    // Check if inference is taking too long
    double max_time = this->get_parameter("max_inference_time_ms").as_double();
    if (inference_time_ms > max_time) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Inference took %.2f ms (exceeds max of %.2f ms) - may not keep up with camera rate",
                           inference_time_ms, max_time);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Inference completed in %.2f ms", inference_time_ms);
    
#else
    RCLCPP_ERROR(this->get_logger(), "ONNX Runtime not available");
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

void ImitationLearningNode::stats_report_timer_callback()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
    now - stats_.last_stats_report).count();
  
  if (elapsed > 0 && stats_.total_inferences > 0) {
    uint64_t total = stats_.total_inferences.load();
    double total_time = stats_.total_inference_time_ms.load();
    double min_time = stats_.min_inference_time_ms.load();
    double max_time = stats_.max_inference_time_ms.load();
    uint64_t dropped = stats_.dropped_frames.load();
    
    double avg_time = total_time / total;
    double inference_rate = static_cast<double>(total) / elapsed;
    
    RCLCPP_INFO(this->get_logger(), 
                "=== Inference Performance Stats (last %ld seconds) ===", elapsed);
    RCLCPP_INFO(this->get_logger(), 
                "  Total inferences: %lu (%.2f Hz)", total, inference_rate);
    RCLCPP_INFO(this->get_logger(), 
                "  Avg inference time: %.2f ms", avg_time);
    RCLCPP_INFO(this->get_logger(), 
                "  Min inference time: %.2f ms", min_time);
    RCLCPP_INFO(this->get_logger(), 
                "  Max inference time: %.2f ms", max_time);
    RCLCPP_INFO(this->get_logger(), 
                "  Dropped frames: %lu", dropped);
    
    // Check if we're keeping up
    double expected_rate = publish_rate_;
    if (inference_rate < expected_rate * 0.9) {
      RCLCPP_WARN(this->get_logger(), 
                  "  ⚠ WARNING: Inference rate (%.2f Hz) is below expected rate (%.2f Hz)",
                  inference_rate, expected_rate);
    } else {
      RCLCPP_INFO(this->get_logger(), 
                  "  ✓ Inference rate is sufficient for %.1f Hz camera", expected_rate);
    }
    
    // Reset stats for next period
    stats_.total_inferences = 0;
    stats_.total_inference_time_ms = 0.0;
    stats_.min_inference_time_ms = 1000.0;
    stats_.max_inference_time_ms = 0.0;
    stats_.dropped_frames = 0;
    stats_.last_stats_report = now;
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ImitationLearningNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
