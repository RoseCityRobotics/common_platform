#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H
#include "setup.h"
#include <Arduino.h>
#if ROS
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

// Forward declarations
class Motion;
class MotorControl;
class RobotState;

// Context structure for odometry timer callback
struct OdometryContext {
  Motion* motion;
  MotorControl* leftMotor;
  MotorControl* rightMotor;
};

// Combined context structure for ROS operations
struct RosContext {
  RobotState* robotState;
  OdometryContext odomContext;
  rcl_clock_t clock;
  float imu_yaw;  // Current IMU heading (radians)
  float initial_mag_x;  // Initial magnetometer X reading for heading correction
  float initial_mag_y;  // Initial magnetometer Y reading for heading correction
  bool mag_calibrated;  // Flag indicating if magnetometer has been calibrated
};

// ROS Agent States
enum RosAgentStatus {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

void setup_ROS();
void handleRosAgentState(void *context);
void resetOdomWithMagnetometerCalibration(void *context);
#endif

#endif