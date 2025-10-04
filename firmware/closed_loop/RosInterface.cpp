#include "RosInterface.h"
#include "RobotState.h"
#include "Motion.h"
#include "MotorControl.h"

#if ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rcl_timer_t odom_timer;
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

  // Set timestamp
  odom_msg.header.stamp.sec = uxr_millis() / 1000;
  odom_msg.header.stamp.nanosec = (uxr_millis() % 1000) * 1000000;
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = 4;
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = 8;

  // Set pose
  odom_msg.pose.pose.position.x = global_ros_context->odomContext.motion->getX();
  odom_msg.pose.pose.position.y = global_ros_context->odomContext.motion->getY();
  odom_msg.pose.pose.position.z = 0.0;

  // Convert theta to quaternion
  float theta = global_ros_context->odomContext.motion->getTheta();
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
  rcl_ret_t ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
  if (ret != RCL_RET_OK) {
    SERIAL_OUT.print("Failed to publish odometry: ");
    SERIAL_OUT.println(ret);
  } else {
#if PRINT_MOVES > 1
    SERIAL_OUT.println("Odometry published successfully");
#endif
  }
}

// Create ROS entities
bool create_entities(void *context) {
  allocator = rcl_get_default_allocator();
  
  // Set global context for timer callback
  global_ros_context = (RosContext*)context;

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_arduino_node", "rcr001", &support));
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  RCCHECK(rclc_timer_init_default(
      &odom_timer, &support, RCL_MS_TO_NS(50), odom_timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, &subscriber, &msg, &subscription_callback, context,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  return true;
}

// Destroy ROS entities
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_timer_fini(&odom_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
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
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
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
