// micro-ROS dual USB-serial Teensy sketch
// - Uses USB Serial (Serial) as the micro-ROS transport
// - Subscribes to /cmd_vel (geometry_msgs/Twist)
// - Echoes received messages to the second USB CDC port (SerialUSB1)

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.h>

// Ensure your Teensy is configured for "USB Type: Dual Serial" so Serial and SerialUSB1 are available.

// --- Agent connection state machine ---
enum AgentStatus {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

static AgentStatus agentStatus = WAITING_AGENT;

// --- ROS entities ---
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t sub_cmd_vel; // "/cmd_vel"
static geometry_msgs__msg__Twist msg_cmd_vel;

// --- Macros (same style as micro-ROS examples) ---
#define RCCHECK(fn)                                   \
  {                                                  \
    rcl_ret_t temp_rc = fn;                          \
    if ((temp_rc != RCL_RET_OK)) {                   \
      SerialUSB1.printf("RCL error %d at %s:%d\n",  \
                        (int)temp_rc, __FILE__, __LINE__); \
      return false;                                  \
    }                                                \
  }

#define EXECUTE_EVERY_N_MS(MS, X)                     \
  do {                                               \
    static volatile int64_t init = -1;               \
    if (init == -1) {                                \
      init = uxr_millis();                            \
    }                                                \
    if (uxr_millis() - init > (MS)) {                \
      X;                                             \
      init = uxr_millis();                           \
    }                                                \
  } while (0)

// --- Subscription callback ---
static void cmd_vel_callback(const void *msgin, void * /*context*/) {
  const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msgin;
  // Echo to the secondary USB CDC port
  SerialUSB1.print("Twist Received: LinX=");
  SerialUSB1.print((double)tw->linear.x, 6);
  SerialUSB1.print(" AngZ=");
  SerialUSB1.println((double)tw->angular.z, 6);
}

// --- Create/destroy entities ---
static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_dual_serial_node", "", &support));

  // Use RELIABLE QoS to match common /cmd_vel publishers
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.depth = 10;
  // Make subscriber durable to receive last latched sample if publisher is transient local
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  // Subscribe to absolute topic name
  RCCHECK(rclc_subscription_init(
      &sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel", &qos));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
  return true;
}

static void destroy_entities() {
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // Start both USB CDC ports
  Serial.begin(115200);
  SerialUSB1.begin(115200);
  delay(100);

  SerialUSB1.println("micro-ROS Dual-Serial Echo starting...");

  // Use default USB Serial as the micro-ROS transport
  set_microros_transports();

  // Prepare message memory
  geometry_msgs__msg__Twist__init(&msg_cmd_vel);

  agentStatus = WAITING_AGENT;
}

void loop() {
  switch (agentStatus) {
    case WAITING_AGENT:
      // Poll for agent availability over serial
      EXECUTE_EVERY_N_MS(500, {
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          agentStatus = AGENT_AVAILABLE;
        } else {
          SerialUSB1.println("ROS: WAITING_AGENT");
        }
      });
      break;

    case AGENT_AVAILABLE:
      SerialUSB1.println("ROS: AGENT_AVAILABLE -> creating entities");
      if (create_entities()) {
        agentStatus = AGENT_CONNECTED;
        SerialUSB1.println("ROS: AGENT_CONNECTED");
      // Optional: wait until at least one publisher is matched to /cmd_vel
      size_t pub_count = 0;
      uint32_t t0 = millis();
      while (millis() - t0 < 1000) { // wait up to 1s
        if (rcl_subscription_get_publisher_count(&sub_cmd_vel, &pub_count) == RCL_RET_OK && pub_count > 0) {
          break;
        }
        delay(50);
      }
      SerialUSB1.print("/cmd_vel matched publishers: ");
      SerialUSB1.println((int)pub_count);
      } else {
        destroy_entities();
        agentStatus = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED: {
      // Periodically check connection liveness and spin executor
      EXECUTE_EVERY_N_MS(200, {
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          agentStatus = AGENT_DISCONNECTED;
        }
      });
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    } break;

    case AGENT_DISCONNECTED:
      SerialUSB1.println("ROS: AGENT_DISCONNECTED -> destroying entities");
      destroy_entities();
      agentStatus = WAITING_AGENT;
      break;
  }
}
