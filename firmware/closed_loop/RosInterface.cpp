#include "RosInterface.h"
#include "RobotState.h"
#include "Motion.h"
#include "MotorControl.h"

#include <Wire.h>  // For I2C communication with IMU
#include <rmw/qos_profiles.h>
#include <cstring>
#include <time.h>
#include <math.h>

// Single definition of the shared IMU address
const uint8_t MPU9250_ADDR = 0x68;

#if ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

RosAgentStatus currentRosAgentStatus = WAITING_AGENT;
RosContext* global_ros_context = nullptr;

// ROS-specific Macros
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop("Failed status on line %d: %d. Aborting.\n", __LINE__,        \
                 (int)temp_rc);                                                \
      return false;                                                            \
    }                                                                          \
  }
#define RCCHECK_VOID(fn)                                                       \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop("Failed status on line %d: %d. Aborting.\n", __LINE__,        \
                 (int)temp_rc);                                                \
      return;                                                                  \
    }                                                                          \
  }
#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// ROS-specific function: Error loop
void error_loop(const char *msg, int line, int rc) {
  SERIAL_OUT.printf("ROS Error at %s:%d code %d\n", __FILE__, line, rc);
  while (1) {
    // TODO: Figure out what to do with LED_PIN
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Twist message callback
void subscription_callback(const void *msgin, void *context) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  RosContext *ros_ctx = (RosContext *)context;
  RobotState *robotState_ptr = ros_ctx->robotState;

  // Directly update robot state from twist message
  robotState_ptr->targetLinearVelocity = msg->linear.x;
  robotState_ptr->targetAngularVelocity = msg->angular.z;
  robotState_ptr->cmdDrive =
      (robotState_ptr->targetLinearVelocity != 0.0f ||
       robotState_ptr->targetAngularVelocity != 0.0f); // Update move flag

  // TODO: Figure out what to do with LED_PIN
  // digitalWrite(LED_PIN, (robotState.targetLinearVelocity == 0 &&
  //                        robotState.targetAngularVelocity == 0)
  //                           ? LOW
  //                           : HIGH);

#if PRINT_MOVES
  SERIAL_OUT.print("Twist Received: LinX=");
  SERIAL_OUT.print(robotState_ptr->targetLinearVelocity);
  SERIAL_OUT.print(" AngZ=");
  SERIAL_OUT.println(robotState_ptr->targetAngularVelocity);
#endif
}


// Timer callback for publishing odometry
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  
#if PRINT_MOVES > 1
  SERIAL_OUT.println("=== ODOM TIMER CALLBACK ===");
#endif
  
  if (!global_ros_context || !global_ros_context->odomContext.motion || !global_ros_context->odomContext.leftMotor || !global_ros_context->odomContext.rightMotor) {
#if PRINT_MOVES > 1
    SERIAL_OUT.println("ERROR: Missing ROS context or motion/motor objects");
#endif
    return;
  }

  // Set timestamp - use ROS system time
  rcl_time_point_value_t now;
  rcl_ret_t ret = rcl_clock_get_now(&global_ros_context->clock, &now);
  if (ret == RCL_RET_OK) {
    odom_msg.header.stamp.sec = now / 1000000000;  // Convert nanoseconds to seconds
    odom_msg.header.stamp.nanosec = now % 1000000000;  // Get remaining nanoseconds
  } else {
    // Fallback to system time if ROS clock fails
    odom_msg.header.stamp.sec = time(NULL);
    odom_msg.header.stamp.nanosec = 0;
  }
  
  // Set frame IDs - use relative frame IDs since node is already namespaced
  const char* frame_id_str = "odom";
  const char* child_frame_id_str = "base_link";
  odom_msg.header.frame_id.data = (char*)frame_id_str;
  odom_msg.header.frame_id.size = strlen(frame_id_str) + 1;  // Include null terminator
  odom_msg.child_frame_id.data = (char*)child_frame_id_str;
  odom_msg.child_frame_id.size = strlen(child_frame_id_str) + 1;  // Include null terminator

  // Update position using IMU heading if calibrated
  // The Motion class accumulates position based on encoder differential
  // We need to recalculate x,y based on the IMU heading
  static float last_theta = 0.0f;
  static float accumulated_x = 0.0f;
  static float accumulated_y = 0.0f;
  static int32_t last_left_count = 0;
  static int32_t last_right_count = 0;
  static bool first_update = true;
  static bool was_calibrated = false;
  
  // Reset accumulated position if calibration state changed
  if (global_ros_context->mag_calibrated != was_calibrated) {
    if (global_ros_context->mag_calibrated) {
      // Just calibrated - reset accumulated position and encoder tracking
      accumulated_x = 0.0f;
      accumulated_y = 0.0f;
      first_update = true;
      // Initialize encoder tracking from current (reset) values
      last_left_count = global_ros_context->odomContext.leftMotor->getCounter();
      last_right_count = global_ros_context->odomContext.rightMotor->getCounter();
      last_theta = 0.0f;  // Reset theta tracking
    }
    was_calibrated = global_ros_context->mag_calibrated;
  }
  
  // Use IMU heading if available and calibrated, otherwise fall back to encoder-based theta
  float theta;
  if (global_ros_context->mag_calibrated) {
    // Use IMU heading (already corrected by initial magnetometer reading)
    theta = global_ros_context->imu_yaw;
    // If this is the first update after calibration, ensure theta is 0
    if (first_update) {
      theta = 0.0f;
      last_theta = 0.0f;
    }
  } else {
    // Fall back to encoder-based theta
    theta = global_ros_context->odomContext.motion->getTheta();
  }
  
  float x, y;
  if (global_ros_context->mag_calibrated) {
    // Get current encoder counts
    int32_t left_count = global_ros_context->odomContext.leftMotor->getCounter();
    int32_t right_count = global_ros_context->odomContext.rightMotor->getCounter();
    
    if (!first_update) {
      // Calculate distance traveled from encoder counts
      // Use the same calculation as Motion class
      float leftTravel = (left_count - last_left_count) * MotorControl::METERS_PER_COUNT;
      float rightTravel = (right_count - last_right_count) * MotorControl::METERS_PER_COUNT;
      float distance = (rightTravel + leftTravel) / 2.0f;
      
      if (fabs(distance) > 1e-6f) {  // Only update if there's significant movement
        // Use average of current and previous IMU heading for smoother integration
        float avg_theta = (theta + last_theta) / 2.0f;
        
        // Accumulate position using IMU heading
        accumulated_x += distance * cos(avg_theta);
        accumulated_y += distance * sin(avg_theta);
      }
    } else {
      // First update after calibration - initialize encoder tracking
      // Ensure theta and last_theta are both 0 after reset
      theta = 0.0f;
      last_theta = 0.0f;
      first_update = false;
    }
    
    // Update encoder tracking for next iteration
    last_left_count = left_count;
    last_right_count = right_count;
    x = accumulated_x;
    y = accumulated_y;
  } else {
    // Use encoder-based position
    x = global_ros_context->odomContext.motion->getX();
    y = global_ros_context->odomContext.motion->getY();
    // Reset accumulated position tracking when not calibrated
    last_left_count = global_ros_context->odomContext.leftMotor->getCounter();
    last_right_count = global_ros_context->odomContext.rightMotor->getCounter();
    accumulated_x = x;
    accumulated_y = y;
    first_update = true;
  }
  last_theta = theta;
  
  // Set pose
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  // Convert theta to quaternion
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

  // Set twist (linear and angular velocities)
  float leftSpeed = global_ros_context->odomContext.leftMotor->getSpeed();
  float rightSpeed = global_ros_context->odomContext.rightMotor->getSpeed();
  float linearVel = (leftSpeed + rightSpeed) / 2.0;
  float angularVel = (rightSpeed - leftSpeed) / Motion::TRACK_WIDTH;

  odom_msg.twist.twist.linear.x = linearVel;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = angularVel;

  // Debug output for odometry data
#if PRINT_MOVES > 1
  SERIAL_OUT.print("Position: x=");
  SERIAL_OUT.print(odom_msg.pose.pose.position.x, 3);
  SERIAL_OUT.print(" y=");
  SERIAL_OUT.print(odom_msg.pose.pose.position.y, 3);
  SERIAL_OUT.print(" theta=");
  SERIAL_OUT.print(theta * 180.0 / M_PI, 1);
  SERIAL_OUT.print("°");
  SERIAL_OUT.print(" | Velocities: lin=");
  SERIAL_OUT.print(linearVel, 3);
  SERIAL_OUT.print(" ang=");
  SERIAL_OUT.print(angularVel, 3);
  SERIAL_OUT.print(" | Motor speeds: L=");
  SERIAL_OUT.print(leftSpeed, 3);
  SERIAL_OUT.print(" R=");
  SERIAL_OUT.print(rightSpeed, 3);
  SERIAL_OUT.println();
#endif

  // Publish the message
  rcl_ret_t publish_ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
  if (publish_ret != RCL_RET_OK) {
    SERIAL_OUT.print("Failed to publish odometry: ");
    SERIAL_OUT.print(publish_ret);
    SERIAL_OUT.print(" (");
    switch(publish_ret) {
      case RCL_RET_ERROR: SERIAL_OUT.print("RCL_RET_ERROR"); break;
      case RCL_RET_BAD_ALLOC: SERIAL_OUT.print("RCL_RET_BAD_ALLOC"); break;
      case RCL_RET_INVALID_ARGUMENT: SERIAL_OUT.print("RCL_RET_INVALID_ARGUMENT"); break;
      case RCL_RET_PUBLISHER_INVALID: SERIAL_OUT.print("RCL_RET_PUBLISHER_INVALID"); break;
      default: SERIAL_OUT.print("Unknown error"); break;
    }
    SERIAL_OUT.println(")");
  } else {
#if PRINT_MOVES > 1
    SERIAL_OUT.println("Odometry published successfully");
#endif
  }
}

// IMU reading function - reads MPU9250 data (accelerometer, gyroscope, and magnetometer)
bool readIMU(float* accel_x, float* accel_y, float* accel_z,
              float* gyro_x, float* gyro_y, float* gyro_z,
              float* mag_x, float* mag_y, float* mag_z) {
  const uint8_t AK8963_ADDR = 0x0C;  // Magnetometer address (accessed via passthrough)
  const uint8_t ACCEL_XOUT_H = 0x3B;
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  uint8_t tx_error = Wire.endTransmission(false);
  if (tx_error != 0) {
    static int comm_error_count = 0;
    if (++comm_error_count % 250 == 0) {  // Log every 250 errors (~5 seconds)
      SERIAL_OUT.print("IMU I2C transmission error: ");
      SERIAL_OUT.println(tx_error);
    }
    return false;  // Communication error
  }
  
  // Request 14 bytes: 6 accel, 2 temp, 6 gyro
  uint8_t bytes_read = Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14, (uint8_t)true);
  if (bytes_read != 14) {
    static int read_error_count = 0;
    if (++read_error_count % 250 == 0) {  // Log every 250 errors
      SERIAL_OUT.print("IMU read error: expected 14 bytes, got ");
      SERIAL_OUT.println(bytes_read);
    }
    return false;  // Not enough data
  }
  
  // Read accelerometer data (signed 16-bit, big-endian)
  int16_t accel_raw[3];
  accel_raw[0] = (Wire.read() << 8) | Wire.read();  // X
  accel_raw[1] = (Wire.read() << 8) | Wire.read();  // Y
  accel_raw[2] = (Wire.read() << 8) | Wire.read();  // Z
  
  // Skip temperature (2 bytes)
  Wire.read();
  Wire.read();
  
  // Read gyroscope data (signed 16-bit, big-endian)
  int16_t gyro_raw[3];
  gyro_raw[0] = (Wire.read() << 8) | Wire.read();  // X
  gyro_raw[1] = (Wire.read() << 8) | Wire.read();  // Y
  gyro_raw[2] = (Wire.read() << 8) | Wire.read();  // Z
  
  // Convert to physical units
  // MPU9250 default: ±2g for accel (16384 LSB/g), ±250°/s for gyro (131 LSB/°/s)
  const float ACCEL_SCALE = 9.80665f / 16384.0f;  // Convert to m/s² (assuming ±2g range)
  const float GYRO_SCALE = (M_PI / 180.0f) / 131.0f;  // Convert to rad/s (assuming ±250°/s range)
  
  *accel_x = accel_raw[0] * ACCEL_SCALE;
  *accel_y = accel_raw[1] * ACCEL_SCALE;
  *accel_z = accel_raw[2] * ACCEL_SCALE;
  
  *gyro_x = gyro_raw[0] * GYRO_SCALE;
  *gyro_y = gyro_raw[1] * GYRO_SCALE;
  *gyro_z = gyro_raw[2] * GYRO_SCALE;
  
  // Read magnetometer (AK8963) - accessed via I2C passthrough
  // Note: Passthrough should be enabled during setup, not here
  // Make magnetometer read optional - if it fails, just set to zero and continue
  *mag_x = 0.0f;
  *mag_y = 0.0f;
  *mag_z = 0.0f;
  
  // Try to read magnetometer, but don't fail if it doesn't work
  // Check if magnetometer is ready (ST1 register)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x02);  // ST1 register
  if (Wire.endTransmission(false) != 0) {
    // Communication failed, skip magnetometer
    return true;  // Still return true since accel/gyro succeeded
  }
  
  uint8_t bytes_read_mag = Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)1, (uint8_t)true);
  if (bytes_read_mag == 0 || !Wire.available()) {
    // No data available, skip magnetometer
    return true;  // Still return true since accel/gyro succeeded
  }
  uint8_t st1 = Wire.read();
  
  if (!(st1 & 0x01)) {
    // Data not ready, skip magnetometer
    return true;  // Still return true since accel/gyro succeeded
  }
  
  // Read magnetometer data (7 bytes: 6 data + 1 status)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03);  // HXL register (start of magnetometer data)
  if (Wire.endTransmission(false) != 0) {
    // Communication failed, skip magnetometer
    return true;  // Still return true since accel/gyro succeeded
  }
  
  bytes_read_mag = Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7, (uint8_t)true);
  if (bytes_read_mag != 7 || Wire.available() < 7) {
    // Not enough data, skip magnetometer
    return true;  // Still return true since accel/gyro succeeded
  }
  
  // Read magnetometer data (signed 16-bit, little-endian for AK8963)
  int16_t mag_raw[3];
  mag_raw[0] = Wire.read() | (Wire.read() << 8);  // X (little-endian)
  mag_raw[1] = Wire.read() | (Wire.read() << 8);  // Y
  mag_raw[2] = Wire.read() | (Wire.read() << 8);  // Z
  Wire.read();  // Skip ST2 register
  
  // Convert to Tesla (μT)
  // AK8963 default: ±4800μT range, 16-bit (32768 LSB full scale)
  // Sensitivity: 4912 LSB/μT for ±4800μT range
  const float MAG_SCALE = 1.0f / 4912.0f;  // Convert to μT, then to Tesla
  
  *mag_x = mag_raw[0] * MAG_SCALE * 1e-6f;  // Convert μT to Tesla
  *mag_y = mag_raw[1] * MAG_SCALE * 1e-6f;
  *mag_z = mag_raw[2] * MAG_SCALE * 1e-6f;
  
  return true;
}

// Timer callback for publishing IMU data
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  
#if PRINT_MOVES > 1
  SERIAL_OUT.println("=== IMU TIMER CALLBACK ===");
#endif
  
  if (!global_ros_context) {
    SERIAL_OUT.println("ERROR: Missing ROS context in IMU callback");
    return;
  }
  
  // Read IMU data
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  
  if (!readIMU(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z, &mag_x, &mag_y, &mag_z)) {
    static int error_count = 0;
    if (++error_count % 50 == 0) {  // Log every 50 errors (~1 second)
      SERIAL_OUT.println("ERROR: Failed to read IMU data - accel/gyro read failed");
    }
    return;
  }
  
  // Set timestamp - use ROS system time
  rcl_time_point_value_t now;
  rcl_ret_t ret = rcl_clock_get_now(&global_ros_context->clock, &now);
  if (ret == RCL_RET_OK) {
    imu_msg.header.stamp.sec = now / 1000000000;  // Convert nanoseconds to seconds
    imu_msg.header.stamp.nanosec = now % 1000000000;  // Get remaining nanoseconds
  } else {
    // Fallback to system time if ROS clock fails
    imu_msg.header.stamp.sec = time(NULL);
    imu_msg.header.stamp.nanosec = 0;
  }
  
  // Set frame ID - must be set every time since it's a string pointer
  // Use a static string to ensure the pointer remains valid
  static const char* frame_id_str = "imu_link";
  imu_msg.header.frame_id.data = (char*)frame_id_str;
  imu_msg.header.frame_id.size = strlen(frame_id_str) + 1;  // Include null terminator
  imu_msg.header.frame_id.capacity = strlen(frame_id_str) + 1;  // Set capacity as well
  
  // Apply gyroscope bias compensation if calibrated
  float corrected_gyro_z = gyro_z;
  if (global_ros_context && global_ros_context->gyro_bias_calibrated) {
    corrected_gyro_z = gyro_z - global_ros_context->gyro_z_bias;
    
    // Apply deadband to zero out very small angular velocities (gyro noise)
    // This prevents false rotation detection when robot is stationary
    const float GYRO_DEADBAND = 0.01f;  // ~0.57 deg/s threshold
    if (fabs(corrected_gyro_z) < GYRO_DEADBAND) {
      corrected_gyro_z = 0.0f;
    }
  }
  
  // Set angular velocity (from gyroscope, with bias compensation)
  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = corrected_gyro_z;
  
  // Set linear acceleration (from accelerometer)
  // Note: This includes gravity. For true linear acceleration, gravity should be subtracted
  // based on orientation, but for simplicity we'll publish raw accelerometer data
  imu_msg.linear_acceleration.x = accel_x;
  imu_msg.linear_acceleration.y = accel_y;
  imu_msg.linear_acceleration.z = accel_z;
  
  // For orientation, use magnetometer for absolute yaw (compass heading)
  // This provides drift-free heading compared to gyroscope integration
  static float yaw = 0.0f;
  static uint32_t last_imu_time = 0;
  uint32_t current_time = millis();
  
  // Reset static yaw when requested (e.g., after calibration)
  if (global_ros_context && global_ros_context->imu_yaw_reset_requested) {
    // Reset yaw and timing
    yaw = 0.0f;
    last_imu_time = 0;  // Force first reading to use magnetometer directly
    global_ros_context->imu_yaw_reset_requested = false;  // Clear the request flag
  }
  
  // Calculate yaw from magnetometer (compass heading)
  // Magnetometer gives us absolute heading in the horizontal plane
  float mag_yaw = 0.0f;
  if (mag_x != 0.0f || mag_y != 0.0f) {
    // Apply initial magnetometer correction if calibrated
    float corrected_mag_x = mag_x;
    float corrected_mag_y = mag_y;
    if (global_ros_context && global_ros_context->mag_calibrated) {
      // Subtract initial magnetometer reading to get relative heading
      corrected_mag_x = mag_x - global_ros_context->initial_mag_x;
      corrected_mag_y = mag_y - global_ros_context->initial_mag_y;
    }
    
    // Calculate heading from corrected magnetometer (atan2 gives angle in XY plane)
    // Note: This assumes magnetometer X/Y are in the horizontal plane
    // If both corrected values are very close to zero (just after calibration), use 0
    float mag_magnitude = sqrt(corrected_mag_x * corrected_mag_x + corrected_mag_y * corrected_mag_y);
    if (mag_magnitude < 1e-6f) {
      // Very small magnitude - treat as zero (just calibrated, no rotation)
      mag_yaw = 0.0f;
    } else {
      mag_yaw = atan2(corrected_mag_y, corrected_mag_x);
    }
    
    // If we have a previous yaw from gyro, use complementary filter
    // to combine smooth gyro updates with absolute magnetometer reference
    if (last_imu_time > 0) {
      float dt = (current_time - last_imu_time) / 1000.0f;  // Convert to seconds
      // Use corrected gyro_z (with bias compensation) for integration
      float gyro_z_corrected = gyro_z;
      if (global_ros_context && global_ros_context->gyro_bias_calibrated) {
        gyro_z_corrected = gyro_z - global_ros_context->gyro_z_bias;
      }
      float gyro_yaw = yaw + gyro_z_corrected * dt;  // Integrate gyroscope
      
      // Normalize angles to [-pi, pi]
      while (gyro_yaw > M_PI) gyro_yaw -= 2.0f * M_PI;
      while (gyro_yaw < -M_PI) gyro_yaw += 2.0f * M_PI;
      while (mag_yaw > M_PI) mag_yaw -= 2.0f * M_PI;
      while (mag_yaw < -M_PI) mag_yaw += 2.0f * M_PI;
      
      // Complementary filter: 95% gyro (smooth), 5% magnetometer (absolute reference)
      // This corrects gyro drift while maintaining smooth updates
      float alpha = 0.95f;  // Gyro weight
      float diff = mag_yaw - gyro_yaw;
      // Handle wrap-around
      if (diff > M_PI) diff -= 2.0f * M_PI;
      if (diff < -M_PI) diff += 2.0f * M_PI;
      yaw = gyro_yaw + (1.0f - alpha) * diff;
    } else {
      // First reading, use magnetometer directly
      yaw = mag_yaw;
    }
  } else {
    // No magnetometer data, fall back to gyroscope integration
    if (last_imu_time > 0) {
      float dt = (current_time - last_imu_time) / 1000.0f;
      // Use corrected gyro_z (with bias compensation) for integration
      float gyro_z_corrected = gyro_z;
      if (global_ros_context && global_ros_context->gyro_bias_calibrated) {
        gyro_z_corrected = gyro_z - global_ros_context->gyro_z_bias;
      }
      yaw += gyro_z_corrected * dt;
      while (yaw > M_PI) yaw -= 2.0f * M_PI;
      while (yaw < -M_PI) yaw += 2.0f * M_PI;
    }
  }
  last_imu_time = current_time;
  
  // Store yaw in global context for odom callback
  if (global_ros_context) {
    global_ros_context->imu_yaw = yaw;
  }
  
  // Convert yaw to quaternion (assuming robot is on flat ground, roll=0, pitch=0)
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = sin(yaw / 2.0f);
  imu_msg.orientation.w = cos(yaw / 2.0f);
  
  // Set covariance matrices
  // Cartographer requires linear_acceleration_covariance[0] != -1
  // -1.0 means data not available, 0.0 means data available but covariance unknown
  // We provide linear acceleration and angular velocity, so set those to 0.0
  // For orientation, we provide it but with less confidence, so use small positive values
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;  // Orientation available (yaw from mag+gyro)
    imu_msg.angular_velocity_covariance[i] = 0.0;  // Angular velocity available
    imu_msg.linear_acceleration_covariance[i] = 0.0;  // Linear acceleration available (required by Cartographer)
  }
  
  // Set diagonal elements to indicate data quality
  // For MPU9250, typical noise levels:
  // Linear acceleration: ~0.01 m/s² noise -> variance ~0.0001
  // Angular velocity: ~0.01 rad/s noise -> variance ~0.0001
  // Orientation (yaw): ~0.05 rad (~3°) uncertainty -> variance ~0.0025
  imu_msg.linear_acceleration_covariance[0] = 0.01;  // X variance (m/s²)²
  imu_msg.linear_acceleration_covariance[4] = 0.01;  // Y variance
  imu_msg.linear_acceleration_covariance[8] = 0.01;  // Z variance
  
  imu_msg.angular_velocity_covariance[0] = 0.01;  // X variance (rad/s)²
  imu_msg.angular_velocity_covariance[4] = 0.01;  // Y variance
  imu_msg.angular_velocity_covariance[8] = 0.01;  // Z variance
  
  imu_msg.orientation_covariance[0] = 0.01;  // Roll variance (rad²) - not used (assumed 0)
  imu_msg.orientation_covariance[4] = 0.01;  // Pitch variance (rad²) - not used (assumed 0)
  imu_msg.orientation_covariance[8] = 0.0025;  // Yaw variance (rad²) - ~3° uncertainty
  
  // Ensure header is properly set (frame_id was set earlier)
  // Make sure frame_id data pointer is valid
  if (imu_msg.header.frame_id.data == NULL) {
    SERIAL_OUT.println("ERROR: IMU frame_id.data is NULL!");
    return;
  }
  
  // Debug output
#if PRINT_MOVES > 1
  SERIAL_OUT.print("IMU: Accel(");
  SERIAL_OUT.print(accel_x, 3);
  SERIAL_OUT.print(", ");
  SERIAL_OUT.print(accel_y, 3);
  SERIAL_OUT.print(", ");
  SERIAL_OUT.print(accel_z, 3);
  SERIAL_OUT.print(") Gyro(");
  SERIAL_OUT.print(gyro_x * 180.0f / M_PI, 1);
  SERIAL_OUT.print("°, ");
  SERIAL_OUT.print(gyro_y * 180.0f / M_PI, 1);
  SERIAL_OUT.print("°, ");
  SERIAL_OUT.print(gyro_z * 180.0f / M_PI, 1);
  SERIAL_OUT.print("°) Mag(");
  SERIAL_OUT.print(mag_x * 1e6f, 1);  // Convert to μT for display
  SERIAL_OUT.print(", ");
  SERIAL_OUT.print(mag_y * 1e6f, 1);
  SERIAL_OUT.print(", ");
  SERIAL_OUT.print(mag_z * 1e6f, 1);
  SERIAL_OUT.print("μT) Yaw=");
  SERIAL_OUT.print(yaw * 180.0f / M_PI, 1);
  SERIAL_OUT.println("°");
#endif
  
  // Publish the message
  rcl_ret_t publish_ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
  if (publish_ret != RCL_RET_OK) {
    static int pub_error_count = 0;
    if (++pub_error_count % 50 == 0) {  // Log every 50 errors
      SERIAL_OUT.print("ERROR: Failed to publish IMU: ");
      SERIAL_OUT.print(publish_ret);
      SERIAL_OUT.print(" (");
      switch(publish_ret) {
        case RCL_RET_ERROR: SERIAL_OUT.print("RCL_RET_ERROR"); break;
        case RCL_RET_BAD_ALLOC: SERIAL_OUT.print("RCL_RET_BAD_ALLOC"); break;
        case RCL_RET_INVALID_ARGUMENT: SERIAL_OUT.print("RCL_RET_INVALID_ARGUMENT"); break;
        case RCL_RET_PUBLISHER_INVALID: SERIAL_OUT.print("RCL_RET_PUBLISHER_INVALID"); break;
        default: SERIAL_OUT.print("Unknown error"); break;
      }
      SERIAL_OUT.println(")");
    }
  }
}

// Reset odometry and calibrate magnetometer
void resetOdomWithMagnetometerCalibration(void *context) {
  RosContext *ros_ctx = (RosContext *)context;
  if (!ros_ctx) {
    SERIAL_OUT.println("ERROR: No ROS context for odom reset");
    return;
  }
  
  // Read current magnetometer reading
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  
  if (readIMU(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z, &mag_x, &mag_y, &mag_z)) {
    // Store initial magnetometer reading
    ros_ctx->initial_mag_x = mag_x;
    ros_ctx->initial_mag_y = mag_y;
    ros_ctx->mag_calibrated = true;
    ros_ctx->imu_yaw = 0.0f;  // Reset yaw to 0
    ros_ctx->imu_yaw_reset_requested = true;  // Request IMU callback to reset static yaw
    
    // Calibrate gyroscope bias (assume robot is stationary during reset)
    // Take multiple readings and average to get bias estimate
    static const int BIAS_SAMPLES = 10;
    float gyro_z_sum = 0.0f;
    for (int i = 0; i < BIAS_SAMPLES; i++) {
      float temp_accel_x, temp_accel_y, temp_accel_z;
      float temp_gyro_x, temp_gyro_y, temp_gyro_z;
      float temp_mag_x, temp_mag_y, temp_mag_z;
      if (readIMU(&temp_accel_x, &temp_accel_y, &temp_accel_z, 
                   &temp_gyro_x, &temp_gyro_y, &temp_gyro_z,
                   &temp_mag_x, &temp_mag_y, &temp_mag_z)) {
        gyro_z_sum += temp_gyro_z;
      }
      delay(10);  // Small delay between samples
    }
    ros_ctx->gyro_z_bias = gyro_z_sum / BIAS_SAMPLES;
    ros_ctx->gyro_bias_calibrated = true;
    
    SERIAL_OUT.print("Magnetometer calibrated: mag_x=");
    SERIAL_OUT.print(mag_x * 1e6f, 2);
    SERIAL_OUT.print("μT, mag_y=");
    SERIAL_OUT.print(mag_y * 1e6f, 2);
    SERIAL_OUT.println("μT");
    SERIAL_OUT.print("Gyroscope bias calibrated: gyro_z_bias=");
    SERIAL_OUT.print(ros_ctx->gyro_z_bias * 180.0f / M_PI, 3);
    SERIAL_OUT.println(" deg/s");
  } else {
    SERIAL_OUT.println("ERROR: Failed to read magnetometer for calibration");
    ros_ctx->mag_calibrated = false;
    return;
  }
  
  // Reset odometry
  if (ros_ctx->odomContext.motion) {
    ros_ctx->odomContext.motion->reset();
  }
  if (ros_ctx->odomContext.leftMotor) {
    ros_ctx->odomContext.leftMotor->resetCounter();
  }
  if (ros_ctx->odomContext.rightMotor) {
    ros_ctx->odomContext.rightMotor->resetCounter();
  }
  
  SERIAL_OUT.println("Odometry reset to position (0, 0) and orientation 0");
  SERIAL_OUT.println("Note: Position will now use IMU heading for accumulation");
}

// Create ROS entities
bool create_entities(void *context) {
  allocator = rcl_get_default_allocator();
  
  // Set global context for timer callback
  global_ros_context = (RosContext*)context;
  
  // Initialize magnetometer calibration fields
  global_ros_context->imu_yaw = 0.0f;
  global_ros_context->initial_mag_x = 0.0f;
  global_ros_context->initial_mag_y = 0.0f;
  global_ros_context->mag_calibrated = false;
  global_ros_context->imu_yaw_reset_requested = false;
  global_ros_context->gyro_z_bias = 0.0f;
  global_ros_context->gyro_bias_calibrated = false;

  // Initialize ROS clock with system time
  RCCHECK(rcl_clock_init(RCL_SYSTEM_TIME, &global_ros_context->clock, &allocator));

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_arduino_node", ROS_NAME, &support));
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  // Get type support for odometry message
  const rosidl_message_type_support_t * odom_type_support = 
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
  
  // Debug: Print type support info
  SERIAL_OUT.print("ROS: Type support name: ");
  SERIAL_OUT.println(odom_type_support->typesupport_identifier);
  
  // Initialize publisher with explicit type support
  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node, odom_type_support, "odom_uros"));
  
  // Initialize odometry message
  nav_msgs__msg__Odometry__init(&odom_msg);
  RCCHECK(rclc_timer_init_default(
      &odom_timer, &support, RCL_MS_TO_NS(50), odom_timer_callback));
  SERIAL_OUT.println("ROS: Odometry timer initialized successfully");
  
  // Get type support for IMU message
  const rosidl_message_type_support_t * imu_type_support = 
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
  
  // Initialize IMU publisher
  RCCHECK(rclc_publisher_init_default(
      &imu_publisher, &node, imu_type_support, "imu_uros/data"));
  SERIAL_OUT.println("ROS: IMU publisher initialized on topic 'imu_uros/data'");
  
  // Initialize IMU message BEFORE timer (order matters for micro-ROS)
  sensor_msgs__msg__Imu__init(&imu_msg);
  
  // Initialize covariance arrays to zero (required for proper message initialization)
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  
  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support, RCL_MS_TO_NS(20), imu_timer_callback));  // 50 Hz IMU update rate
  SERIAL_OUT.println("ROS: IMU timer initialized successfully (50 Hz)");
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));  // Increased to 3 for IMU timer
  SERIAL_OUT.println("ROS: Executor initialized successfully");
  
  RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, &subscriber, &msg, &subscription_callback, context,
      ON_NEW_DATA));
  SERIAL_OUT.println("ROS: Subscription added to executor");
  
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  SERIAL_OUT.println("ROS: Odometry timer added to executor successfully");
  
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  SERIAL_OUT.println("ROS: IMU timer added to executor successfully");
  
  return true;
}

// Destroy ROS entities
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK_VOID(rcl_subscription_fini(&subscriber, &node));
  RCCHECK_VOID(rcl_publisher_fini(&odom_publisher, &node));
  RCCHECK_VOID(rcl_publisher_fini(&imu_publisher, &node));
  RCCHECK_VOID(rcl_timer_fini(&odom_timer));
  RCCHECK_VOID(rcl_timer_fini(&imu_timer));
  RCCHECK_VOID(rclc_executor_fini(&executor));
  RCCHECK_VOID(rcl_node_fini(&node));
  RCCHECK_VOID(rclc_support_fini(&support));
  
  // Clean up clock
  if (global_ros_context) {
    RCCHECK_VOID(rcl_clock_fini(&global_ros_context->clock));
  }
  
  // Cleanup odometry message
  nav_msgs__msg__Odometry__fini(&odom_msg);
  
  // Cleanup IMU message
  sensor_msgs__msg__Imu__fini(&imu_msg);
}

// Handle agent connection state machine
void handleRosAgentState(void *context) {
  RosContext *ros_ctx = (RosContext *)context;
  RobotState *robotState_ptr = ros_ctx->robotState;
  static bool loggedConnected = false; // print-once guard for AGENT_CONNECTED

  switch (currentRosAgentStatus) {

  case WAITING_AGENT:
    // Use ping to discover the agent when not connected
    SERIAL_OUT.println("ROS: WAITING_AGENT");
    loggedConnected = false; // reset on state change away from CONNECTED
    EXECUTE_EVERY_N_MS(500, currentRosAgentStatus =
                                (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                    ? AGENT_AVAILABLE
                                    : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    // Agent is available, so create all the ROS entities
    SERIAL_OUT.println("ROS: AGENT_AVAILABLE -> creating entities");
    loggedConnected = false; // ensure reset before entering CONNECTED
    currentRosAgentStatus =
        (true == create_entities(context)) ? AGENT_CONNECTED : WAITING_AGENT;
    if (currentRosAgentStatus == WAITING_AGENT) {
      destroy_entities(); // Clean up if creation failed
      SERIAL_OUT.println("ROS: create_entities failed, back to WAITING_AGENT");
    };
    break;
  case AGENT_CONNECTED: {
    if (!loggedConnected) {
      SERIAL_OUT.println("ROS: AGENT_CONNECTED");
      loggedConnected = true;
    }
    EXECUTE_EVERY_N_MS(200, currentRosAgentStatus =
                                (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                    ? AGENT_CONNECTED
                                    : AGENT_DISCONNECTED;);
    if (currentRosAgentStatus == AGENT_CONNECTED) {
      rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      if (spin_ret != RCL_RET_OK) {
        SERIAL_OUT.print("ROS: Executor spin error: ");
        SERIAL_OUT.println(spin_ret);
      }
      if (PRINT_MOVES > 1) {
        // Add periodic debug to see if executor is running
        static int executor_debug_count = 0;
        if (++executor_debug_count > 1000) { // Every ~10 seconds
          SERIAL_OUT.println("ROS: Executor is running...");
          executor_debug_count = 0;
        }
      }
    }
  } break;
  case AGENT_DISCONNECTED:
    // Clean up all entities and return to waiting for the agent
    SERIAL_OUT.println("ROS: AGENT_DISCONNECTED -> destroying entities");
    destroy_entities();
    currentRosAgentStatus = WAITING_AGENT;
    // Reset speeds when disconnected
    robotState_ptr->targetLinearVelocity = 0.0f;
    robotState_ptr->targetAngularVelocity = 0.0f;
    robotState_ptr->move = false;
    SERIAL_OUT.println("ROS: targets cleared due to disconnect");
    loggedConnected = false; // allow message again when reconnected
    break;
  default:
    break;
  }
}
void setup_ROS() { set_microros_transports(); }
#endif
