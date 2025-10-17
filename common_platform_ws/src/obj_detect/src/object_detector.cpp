#include "obj_detect/object_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>

using namespace obj_detect;

ObjectDetector::ObjectDetector(const rclcpp::NodeOptions& options)
  : Node("object_detector", options)
{
  // Initialize parameters
  this->declare_parameter("model_path", "/home/rcr/repos/common_platform/models/yolov11n_2cls.hef");
  this->declare_parameter("confidence_threshold", 0.5);
  this->declare_parameter("input_width", 640);
  this->declare_parameter("input_height", 640);
  this->declare_parameter("hailort_log_path", "/tmp");

  model_path_ = this->get_parameter("model_path").as_string();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  input_width_ = this->get_parameter("input_width").as_int();
  input_height_ = this->get_parameter("input_height").as_int();
  
  // Note: HailoRT logging is configured in main() before node initialization
  // This parameter is kept for informational purposes
  std::string log_path = this->get_parameter("hailort_log_path").as_string();
  RCLCPP_INFO(this->get_logger(), "HailoRT log directory: %s (hailort.log will be created here)", log_path.c_str());

  // Initialize class names for 2-class YOLO model
  class_names_ = {"Purple ball", "Green ball"};
  
  // Initialize last publish time (start at epoch to allow first heartbeat immediately)
  last_publish_time_ = rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);

  RCLCPP_INFO(this->get_logger(), "Initializing Object Detector");
  RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);

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
    std::bind(&ObjectDetector::image_callback, this, std::placeholders::_1));

  // Create publisher for detection results
  detection_publisher_ = this->create_publisher<obj_detect::msg::DetectionArray>(
    "detect", 10);

  RCLCPP_INFO(this->get_logger(), "Object detector node initialized");
}

ObjectDetector::~ObjectDetector()
{
  // Cleanup HailoRT resources in reverse order of initialization
#ifdef HAVE_HAILORT
  RCLCPP_INFO(this->get_logger(), "Shutting down HailoRT resources");
  
  // Clear vstreams first (they depend on activated network group)
  output_vstreams_.clear();
  input_vstreams_.clear();
  
  // Reset activated network group (depends on configured network group)
  activated_network_group_.reset();
  
  // Reset network group (depends on device)
  configured_network_group_.reset();
  
  // Reset device last
  device_.reset();
  
  RCLCPP_INFO(this->get_logger(), "HailoRT resources cleaned up");
#endif
}

void ObjectDetector::initialize_hailo()
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
    
    // 6. Activate the network group (required before using vstreams)
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

void ObjectDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Convert ROS image to OpenCV Mat in RGB format (HailoRT expects RGB)
    cv::Mat image;
    if (msg->encoding == "rgb8") {
      // Already RGB, just wrap the data
      image = cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
    } else if (msg->encoding == "bgr8") {
      // Convert BGR to RGB
      cv::Mat bgr_image = cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
      cv::cvtColor(bgr_image, image, cv::COLOR_BGR2RGB);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    // Run inference (image is already in RGB format)
    auto detections = run_inference(image);

    // Publish results with heartbeat logic:
    // - Always publish if there are detections
    // - Publish empty message as heartbeat every 2 seconds if no detections
    rclcpp::Time current_time = this->now();
    rclcpp::Duration time_since_last_publish = current_time - last_publish_time_;
    
    bool should_publish = !detections.empty() || (time_since_last_publish.seconds() >= 2.0);
    
    if (should_publish) {
      publish_detections(detections, msg->header);
      last_publish_time_ = current_time;
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
  }
}

std::vector<Detection> ObjectDetector::run_inference(const cv::Mat& image)
{
  std::vector<Detection> detections;

  try {
    // Preprocess image (resize to model input dimensions)
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(input_width_, input_height_));
    
#ifdef HAVE_HAILORT
    if (!input_vstreams_.empty() && !output_vstreams_.empty()) {
      // Run actual HailoRT inference
      RCLCPP_DEBUG(this->get_logger(), "Running HailoRT inference on %dx%d image", 
                   image.cols, image.rows);
      
      // 1. Send RGB data to input vstream (image is already in RGB format)
      auto& input_vstream = input_vstreams_[0];
      size_t input_frame_size = input_vstream->get_frame_size();
      size_t image_size = resized_image.total() * resized_image.elemSize();
      
      if (input_frame_size != image_size) {
        RCLCPP_ERROR(this->get_logger(), "Input size mismatch: expected %zu bytes, got %zu bytes", 
                     input_frame_size, image_size);
        return detections;
      }
      
      auto status = input_vstream->write(hailort::MemoryView(resized_image.data, input_frame_size));
      if (status != HAILO_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to input vstream");
        return detections;
      }
      
      // 2. Read results from output vstream
      std::vector<uint8_t> output_buffer;
      for (auto& output_vstream : output_vstreams_) {
        auto output_info = output_vstream->get_info();
        
        // Use get_frame_size() to get the exact buffer size required by HailoRT
        size_t output_size = output_vstream->get_frame_size();
        output_buffer.resize(output_size);
        
        status = output_vstream->read(hailort::MemoryView(output_buffer.data(), output_buffer.size()));
        if (status != HAILO_SUCCESS) {
          RCLCPP_ERROR(this->get_logger(), "Failed to read from output vstream");
          continue;
        }
        
        // 3. Parse YOLO output and extract detections
        // Note: NMS is already applied by the optimized HEF model
        auto parsed_detections = parse_yolo_output(output_buffer, output_info, image.cols, image.rows);
        detections.insert(detections.end(), parsed_detections.begin(), parsed_detections.end());
      }
      
      RCLCPP_DEBUG(this->get_logger(), "HailoRT inference completed with %zu detections", detections.size());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "HailoRT vstreams not available");
    }
#else
    // Simulation mode - create dummy detections for testing
    RCLCPP_DEBUG(this->get_logger(), "Running simulation inference on %dx%d image", 
                 image.cols, image.rows);
    
    // Create dummy detections for testing
    Detection dummy_detection1;
    dummy_detection1.class_name = "Purple ball";
    dummy_detection1.confidence = 0.85f;
    dummy_detection1.bbox = cv::Rect(100, 100, 200, 200);
    detections.push_back(dummy_detection1);
    
    Detection dummy_detection2;
    dummy_detection2.class_name = "Green ball";
    dummy_detection2.confidence = 0.78f;
    dummy_detection2.bbox = cv::Rect(300, 150, 180, 180);
    detections.push_back(dummy_detection2);
#endif

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
  }

  return detections;
}

void ObjectDetector::publish_detections(const std::vector<Detection>& detections, 
                                       const std_msgs::msg::Header& header)
{
  auto detection_array_msg = obj_detect::msg::DetectionArray();
  
  // Copy header from input image - this preserves the correct frame_id
  // The detections are in the camera coordinate system, so we use the camera's frame_id
  detection_array_msg.header = header;

  for (const auto& detection : detections) {
    obj_detect::msg::Detection detection_msg;
    detection_msg.class_name = detection.class_name;
    detection_msg.confidence = detection.confidence;
    
    // Use x,y,w,h format directly from YOLO model output
    detection_msg.x = detection.bbox.x;
    detection_msg.y = detection.bbox.y;
    detection_msg.width = detection.bbox.width;
    detection_msg.height = detection.bbox.height;
    
    detection_array_msg.detections.push_back(detection_msg);
    
    // Log each detection at INFO level for easy debugging
    RCLCPP_INFO(this->get_logger(), "Detected: %s (confidence: %.2f) at [%d, %d, %dx%d]",
                detection.class_name.c_str(), detection.confidence,
                detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height);
  }

  detection_publisher_->publish(detection_array_msg);
  
  RCLCPP_DEBUG(this->get_logger(), "Published %zu detections", detections.size());
}

#ifdef HAVE_HAILORT
std::vector<Detection> ObjectDetector::parse_yolo_output(const std::vector<uint8_t>& output,
                                                         const hailo_vstream_info_t& shape,
                                                         int orig_width, int orig_height)
{
  std::vector<Detection> detections;
  
  // YOLOv11 output format after NMS: [num_boxes, 6] where 6 = [x, y, w, h, confidence, class]
  // The optimized HEF model already applies NMS, so we just need to parse the output
  
  const float* output_float = reinterpret_cast<const float*>(output.data());
  int num_boxes = shape.shape.height;
  int box_size = shape.shape.width; // Should be 6 or 7 for YOLO
  
  RCLCPP_DEBUG(this->get_logger(), "Parsing YOLO output: num_boxes=%d, box_size=%d, buffer_size=%zu",
               num_boxes, box_size, output.size());
  
  // Preallocate capacity to avoid reallocations
  detections.reserve(num_boxes);
  
  for (int i = 0; i < num_boxes; i++) {
    int offset = i * box_size;
    
    // Extract bounding box coordinates (normalized 0-1)
    float x_center = output_float[offset + 0];
    float y_center = output_float[offset + 1];
    float width = output_float[offset + 2];
    float height = output_float[offset + 3];
    float confidence = output_float[offset + 4];
    int class_id = static_cast<int>(output_float[offset + 5]);
    
    // Filter by confidence threshold
    if (confidence < confidence_threshold_) {
      continue;
    }
    
    // Convert from normalized coordinates to pixel coordinates
    int x = static_cast<int>((x_center - width / 2.0f) * orig_width);
    int y = static_cast<int>((y_center - height / 2.0f) * orig_height);
    int w = static_cast<int>(width * orig_width);
    int h = static_cast<int>(height * orig_height);
    
    // Clamp to image boundaries
    x = std::max(0, std::min(x, orig_width - 1));
    y = std::max(0, std::min(y, orig_height - 1));
    w = std::min(w, orig_width - x);
    h = std::min(h, orig_height - y);
    
    // Determine class name
    std::string class_name;
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
      class_name = class_names_[class_id];
    } else {
      class_name = "Unknown";
    }
    
    // Debug logging to help diagnose class detection issues
    RCLCPP_DEBUG(this->get_logger(), "Detection: class_id=%d, class_name='%s', confidence=%.2f, bbox=[%d,%d,%dx%d]",
                 class_id, class_name.c_str(), confidence, x, y, w, h);
    
    // Emplace detection directly into vector (construct in place, no copy)
    Detection detection;
    detection.class_name = std::move(class_name);
    detection.confidence = confidence;
    detection.bbox = cv::Rect(x, y, w, h);
    detections.emplace_back(std::move(detection));
  }
  
  return detections;
}

#endif

int main(int argc, char* argv[])
{
  // Set HailoRT log directory BEFORE any HailoRT initialization
  // HailoRT expects just the directory path, not the full file path
  // It will create hailort.log in this directory
  setenv("HAILORT_LOGGER_PATH", "/tmp", 1);  // 1 = overwrite if already set
  
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ObjectDetector>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
