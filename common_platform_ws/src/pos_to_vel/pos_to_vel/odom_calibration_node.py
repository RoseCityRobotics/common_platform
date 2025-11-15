#!/usr/bin/env python3

"""
Odometry Calibration Node

This node provides discrete keyboard teleop for calibrating robot odometry.
It subscribes to lidar scan and odom topics, and publishes cmd_vel commands
for discrete movements. It displays calibration information showing how well
the actual readings match expected values.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from evdev import InputDevice, ecodes
import math
import threading
import time
import os
from enum import Enum

KEY_DOWN = 1
KEY_UP = 0
KEY_HOLD = 2

class ActionType(Enum):
  FORWARD = 0  # Up arrow
  LEFT = 1    # Left arrow: turn left 90, then forward
  RIGHT = 2   # Right arrow: turn right 90, then forward
  BACK = 3    # Down arrow: turn 180, then forward

class OdomCalibrationNode(Node):
  def __init__(self):
    super().__init__('odom_calibration_node')
    
    # Parameters
    self.declare_parameter('forward_distance', 0.05)  # 5 cm default
    self.declare_parameter('turn_angle', math.pi / 2.0)  # 90 degrees
    self.declare_parameter('linear_speed', 0.1)  # m/s
    self.declare_parameter('angular_speed', 0.5)  # rad/s
    self.declare_parameter('scan_topic', 'scan')
    self.declare_parameter('odom_topic', 'odom')
    self.declare_parameter('cmd_vel_topic', 'cmd_vel')
    self.declare_parameter('namespace', '')
    self.declare_parameter('device_path', '/dev/input/event0')
    self.declare_parameter('grab_device', True)
    self.declare_parameter('debug', True)
    
    # Get parameters
    self.forward_distance = self.get_parameter('forward_distance').value
    self.turn_angle = self.get_parameter('turn_angle').value
    self.linear_speed = self.get_parameter('linear_speed').value
    self.angular_speed = self.get_parameter('angular_speed').value
    scan_topic = self.get_parameter('scan_topic').value
    odom_topic = self.get_parameter('odom_topic').value
    cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
    namespace = self.get_parameter('namespace').value
    self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
    self.grab_device = bool(self.get_parameter('grab_device').value)
    self.debug = bool(self.get_parameter('debug').value)
    
    # Add namespace if provided
    if namespace:
      scan_topic = f'{namespace}/{scan_topic}'
      odom_topic = f'{namespace}/{odom_topic}'
      cmd_vel_topic = f'{namespace}/{cmd_vel_topic}'
    
    # Publishers
    self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
    
    # Create BEST_EFFORT QoS profile for odometry (to match qos_relay output)
    best_effort_qos = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      durability=DurabilityPolicy.VOLATILE,
      history=HistoryPolicy.KEEP_LAST,
      depth=10
    )
    
    # Subscribers
    self.scan_sub = self.create_subscription(
      LaserScan, scan_topic, self.scan_callback, 10
    )
    self.odom_sub = self.create_subscription(
      Odometry, odom_topic, self.odom_callback, best_effort_qos
    )
    
    # State
    self.current_odom = None
    self.current_scan = None
    self.expected_position = None
    self.expected_orientation = None
    self.start_position = None
    self.start_orientation = None
    self.is_executing_action = False
    self.action_lock = threading.Lock()
    
    # Calibration adjustments (user can modify these)
    self.forward_distance_adjust = 0.0  # meters
    self.turn_angle_adjust = 0.0  # radians
    self.linear_speed_adjust = 0.0  # m/s
    self.angular_speed_adjust = 0.0  # rad/s
    
    # Statistics
    self.action_count = 0
    self.position_errors = []
    self.orientation_errors = []
    
    # Timer for publishing cmd_vel during actions
    self.cmd_timer = self.create_timer(0.1, self.cmd_vel_timer_callback)
    self.cmd_vel_queue = []
    self.check_results_after = None  # Timestamp when to check results
    
    # Keyboard input thread
    self.keyboard_thread = None
    self._stop_keyboard = threading.Event()
    self.start_keyboard_input()
    
    self.get_logger().info('Odometry calibration node started')
    self.get_logger().info(f'Subscribed to: {scan_topic}, {odom_topic}')
    self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
    self.get_logger().info(f'Reading keyboard from: {self.device_path}')
    
    # Print instructions to both stdout and logger
    self.print_instructions()
    self.log_instructions()
  
  def start_keyboard_input(self):
    """Start keyboard input thread"""
    self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
    self.keyboard_thread.start()
  
  def print_instructions(self):
    """Print keyboard control instructions to stdout"""
    print("\n" + "="*60)
    print("ODOMETRY CALIBRATION CONTROLS")
    print("="*60)
    print("Arrow Keys:")
    print("  ↑ (Up)    - Move forward")
    print("  ← (Left)  - Turn left 90° then move forward")
    print("  → (Right) - Turn right 90° then move forward")
    print("  ↓ (Down)  - Turn 180° then move forward")
    print("\nCalibration Adjustments:")
    print("  w/s       - Increase/decrease forward distance")
    print("  a/d       - Increase/decrease turn angle")
    print("  q/e       - Increase/decrease linear speed")
    print("  z/x       - Increase/decrease angular speed")
    print("\nOther:")
    print("  r         - Reset statistics")
    print("  p         - Print current calibration values")
    print("  h         - Show this help")
    print("  Ctrl+C    - Exit")
    print("="*60 + "\n")
  
  def log_instructions(self):
    """Log keyboard control instructions to ROS logger"""
    self.get_logger().info("="*60)
    self.get_logger().info("ODOMETRY CALIBRATION CONTROLS")
    self.get_logger().info("="*60)
    self.get_logger().info("Arrow Keys:")
    self.get_logger().info("  ↑ (Up)    - Move forward")
    self.get_logger().info("  ← (Left)  - Turn left 90° then move forward")
    self.get_logger().info("  → (Right) - Turn right 90° then move forward")
    self.get_logger().info("  ↓ (Down)  - Turn 180° then move forward")
    self.get_logger().info("Calibration Adjustments:")
    self.get_logger().info("  w/s       - Increase/decrease forward distance")
    self.get_logger().info("  a/d       - Increase/decrease turn angle")
    self.get_logger().info("  q/e       - Increase/decrease linear speed")
    self.get_logger().info("  z/x       - Increase/decrease angular speed")
    self.get_logger().info("Other:")
    self.get_logger().info("  r         - Reset statistics")
    self.get_logger().info("  p         - Print current calibration values")
    self.get_logger().info("  h         - Show this help")
    self.get_logger().info("="*60)
  
  def keyboard_input_loop(self):
    """Thread for handling keyboard input from evdev device"""
    try:
      dev = InputDevice(self.device_path)
    except Exception as e:
      self.get_logger().error(f'Cannot open {self.device_path}: {e}')
      self.get_logger().error('Make sure the keyboard dongle is connected and the device path is correct')
      return
    
    try:
      if self.grab_device:
        dev.grab()  # Prevent events from going to the console/X
        self.get_logger().info(f'Grabbed device {self.device_path} exclusively')
    except Exception as e:
      self.get_logger().warn(f'Could not grab device exclusively: {e}')
    
    # Key mapping for calibration controls
    KEYMAP = {
      ecodes.KEY_UP: ('action', ActionType.FORWARD),
      ecodes.KEY_DOWN: ('action', ActionType.BACK),
      ecodes.KEY_LEFT: ('action', ActionType.LEFT),
      ecodes.KEY_RIGHT: ('action', ActionType.RIGHT),
      ecodes.KEY_W: ('forward_adj', +0.001),
      ecodes.KEY_S: ('forward_adj', -0.001),
      ecodes.KEY_A: ('turn_adj', +0.01),
      ecodes.KEY_D: ('turn_adj', -0.01),
      ecodes.KEY_Q: ('linear_speed_adj', +0.01),
      ecodes.KEY_E: ('linear_speed_adj', -0.01),
      ecodes.KEY_Z: ('angular_speed_adj', +0.05),
      ecodes.KEY_X: ('angular_speed_adj', -0.05),
      ecodes.KEY_R: ('reset', None),
      ecodes.KEY_P: ('print', None),
      ecodes.KEY_H: ('help', None),
    }
    
    while not self._stop_keyboard.is_set() and rclpy.ok():
      try:
        for event in dev.read_loop():
          if self._stop_keyboard.is_set():
            break
          
          if event.type == ecodes.EV_KEY:
            keyevent = event.value  # 0=up, 1=down, 2=hold
            code = event.code
            
            # Only process key down events (not key up or hold)
            if keyevent == KEY_DOWN and code in KEYMAP:
              action_type, value = KEYMAP[code]
              
              if self.debug:
                try:
                  key_name = ecodes.KEY[code]
                except Exception:
                  key_name = str(code)
                self.get_logger().debug(f'Key pressed: {key_name}')
              
              if action_type == 'action':
                self.execute_action(value)
              elif action_type == 'forward_adj':
                self.forward_distance_adjust += value
                self.get_logger().info(f'Forward distance adjustment: {self.forward_distance_adjust:.4f} m')
              elif action_type == 'turn_adj':
                self.turn_angle_adjust += value
                self.get_logger().info(f'Turn angle adjustment: {self.turn_angle_adjust:.4f} rad ({math.degrees(self.turn_angle_adjust):.2f}°)')
              elif action_type == 'linear_speed_adj':
                self.linear_speed_adjust += value
                self.get_logger().info(f'Linear speed adjustment: {self.linear_speed_adjust:.4f} m/s')
              elif action_type == 'angular_speed_adj':
                self.angular_speed_adjust += value
                self.get_logger().info(f'Angular speed adjustment: {self.angular_speed_adjust:.4f} rad/s')
              elif action_type == 'reset':
                self.reset_statistics()
              elif action_type == 'print':
                self.print_calibration_values()
              elif action_type == 'help':
                self.print_instructions()
                self.log_instructions()
      except Exception as e:
        if not self._stop_keyboard.is_set():
          self.get_logger().error(f'Error in keyboard input loop: {e}')
          time.sleep(0.1)  # Brief pause before retrying
  
  def execute_action(self, action_type: ActionType):
    """Execute a discrete action"""
    with self.action_lock:
      if self.is_executing_action:
        self.get_logger().warn('Action already in progress, ignoring command')
        return
      
      if self.current_odom is None:
        self.get_logger().warn('No odometry data available yet')
        return
      
      self.is_executing_action = True
      self.action_count += 1
      
      # Record starting position
      self.start_position = (
        self.current_odom.pose.pose.position.x,
        self.current_odom.pose.pose.position.y
      )
      
      # Get starting orientation (yaw from quaternion)
      q = self.current_odom.pose.pose.orientation
      self.start_orientation = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
      )
      
      # Calculate expected final position and orientation
      effective_forward = self.forward_distance + self.forward_distance_adjust
      effective_turn = self.turn_angle + self.turn_angle_adjust
      effective_linear_speed = self.linear_speed + self.linear_speed_adjust
      effective_angular_speed = self.angular_speed + self.angular_speed_adjust
      
      if action_type == ActionType.FORWARD:
        # Just move forward
        expected_theta = self.start_orientation
        expected_x = self.start_position[0] + effective_forward * math.cos(expected_theta)
        expected_y = self.start_position[1] + effective_forward * math.sin(expected_theta)
      elif action_type == ActionType.LEFT:
        # Turn left 90, then forward
        expected_theta = self.start_orientation + effective_turn
        expected_x = self.start_position[0] + effective_forward * math.cos(expected_theta)
        expected_y = self.start_position[1] + effective_forward * math.sin(expected_theta)
      elif action_type == ActionType.RIGHT:
        # Turn right 90, then forward
        expected_theta = self.start_orientation - effective_turn
        expected_x = self.start_position[0] + effective_forward * math.cos(expected_theta)
        expected_y = self.start_position[1] + effective_forward * math.sin(expected_theta)
      elif action_type == ActionType.BACK:
        # Turn 180, then forward (use math.pi for 180 degrees, but allow adjustment)
        back_turn_angle = math.pi + self.turn_angle_adjust
        expected_theta = self.start_orientation + back_turn_angle
        expected_x = self.start_position[0] + effective_forward * math.cos(expected_theta)
        expected_y = self.start_position[1] + effective_forward * math.sin(expected_theta)
      
      self.expected_position = (expected_x, expected_y)
      self.expected_orientation = expected_theta
      
      # Normalize expected_orientation to [-pi, pi]
      while self.expected_orientation > math.pi:
        self.expected_orientation -= 2 * math.pi
      while self.expected_orientation < -math.pi:
        self.expected_orientation += 2 * math.pi
      
      self.get_logger().info(f'Executing action {action_type.name}')
      self.get_logger().info(f'Start: ({self.start_position[0]:.3f}, {self.start_position[1]:.3f}), theta: {math.degrees(self.start_orientation):.1f}°')
      self.get_logger().info(f'Expected: ({expected_x:.3f}, {expected_y:.3f}), theta: {math.degrees(self.expected_orientation):.1f}°')
      self.get_logger().info(f'Using effective values: forward={effective_forward:.4f}m, turn={math.degrees(effective_turn):.2f}°, lin_speed={effective_linear_speed:.3f}m/s, ang_speed={effective_angular_speed:.3f}rad/s')
      
      # Generate command sequence
      commands = self.generate_action_commands(action_type, effective_forward, effective_turn, 
                                                 effective_linear_speed, effective_angular_speed)
      
      # Log the command sequence
      self.get_logger().info('Command sequence:')
      total_duration = 0.0
      for i, (cmd_type, duration, lin_vel, ang_vel) in enumerate(commands):
        total_duration += duration
        if cmd_type == 'turn':
          self.get_logger().info(f'  [{i+1}] {cmd_type}: duration={duration:.3f}s, ang_vel={ang_vel:.3f}rad/s ({math.degrees(ang_vel):.2f}°/s), expected_angle={math.degrees(abs(ang_vel * duration)):.2f}°')
        elif cmd_type == 'forward':
          self.get_logger().info(f'  [{i+1}] {cmd_type}: duration={duration:.3f}s, lin_vel={lin_vel:.3f}m/s, expected_distance={lin_vel * duration:.4f}m')
        else:
          self.get_logger().info(f'  [{i+1}] {cmd_type}: duration={duration:.3f}s')
      self.get_logger().info(f'Total action duration: {total_duration:.3f}s')
      
      # Execute commands
      self.execute_command_sequence(commands)
  
  def generate_action_commands(self, action_type: ActionType, forward_dist: float, 
                               turn_angle: float, lin_speed: float, ang_speed: float):
    """Generate sequence of cmd_vel commands for an action"""
    commands = []
    
    if action_type == ActionType.FORWARD:
      # Just move forward
      duration = forward_dist / lin_speed
      commands.append(('forward', duration, lin_speed, 0.0))
    elif action_type == ActionType.LEFT:
      # Turn left, then forward
      turn_duration = turn_angle / ang_speed
      commands.append(('turn', turn_duration, 0.0, ang_speed))
      forward_duration = forward_dist / lin_speed
      commands.append(('forward', forward_duration, lin_speed, 0.0))
    elif action_type == ActionType.RIGHT:
      # Turn right, then forward
      turn_duration = turn_angle / ang_speed
      commands.append(('turn', turn_duration, 0.0, -ang_speed))
      forward_duration = forward_dist / lin_speed
      commands.append(('forward', forward_duration, lin_speed, 0.0))
    elif action_type == ActionType.BACK:
      # Turn 180, then forward (use math.pi for 180 degrees, but allow adjustment)
      back_turn_angle = math.pi + self.turn_angle_adjust
      turn_duration = abs(back_turn_angle) / ang_speed
      commands.append(('turn', turn_duration, 0.0, ang_speed))
      forward_duration = forward_dist / lin_speed
      commands.append(('forward', forward_duration, lin_speed, 0.0))
    
    # Always end with stop
    commands.append(('stop', 0.5, 0.0, 0.0))
    
    return commands
  
  def execute_command_sequence(self, commands):
    """Execute a sequence of commands"""
    self.cmd_vel_queue = commands.copy()
    self.cmd_start_time = time.time()
    self.current_cmd_index = 0
  
  def cmd_vel_timer_callback(self):
    """Timer callback to publish cmd_vel during action execution"""
    # Check if we need to check results after action completion
    if self.check_results_after is not None and time.time() >= self.check_results_after:
      self.check_calibration_results()
      self.check_results_after = None
    
    if not self.cmd_vel_queue:
      return
    
    current_time = time.time()
    elapsed = current_time - self.cmd_start_time
    
    # Find which command we should be executing
    cumulative_time = 0.0
    active_cmd = None
    
    for i, (cmd_type, duration, lin_vel, ang_vel) in enumerate(self.cmd_vel_queue):
      if elapsed < cumulative_time + duration:
        active_cmd = (cmd_type, lin_vel, ang_vel)
        self.current_cmd_index = i
        break
      cumulative_time += duration
    
    if active_cmd:
      # Publish active command
      twist = Twist()
      twist.linear.x = active_cmd[1]
      twist.angular.z = active_cmd[2]
      self.cmd_vel_pub.publish(twist)
    else:
      # All commands complete
      # Send stop command
      twist = Twist()
      twist.linear.x = 0.0
      twist.angular.z = 0.0
      self.cmd_vel_pub.publish(twist)
      
      # Clear queue and mark action as complete
      self.cmd_vel_queue = []
      with self.action_lock:
        self.is_executing_action = False
      
      # Schedule result check after a short delay for odometry to settle
      self.check_results_after = time.time() + 0.5
  
  def check_calibration_results(self):
    """Check how well actual readings match expected values"""
    if self.current_odom is None or self.expected_position is None:
      self.get_logger().warn('Cannot check results: missing odometry or expected position')
      return
    
    # Get current position
    current_x = self.current_odom.pose.pose.position.x
    current_y = self.current_odom.pose.pose.position.y
    
    # Get current orientation
    q = self.current_odom.pose.pose.orientation
    current_theta = math.atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    
    # Calculate position errors (X and Y separately)
    pos_error_x = current_x - self.expected_position[0]
    pos_error_y = current_y - self.expected_position[1]
    pos_error = math.sqrt(pos_error_x**2 + pos_error_y**2)
    
    # Calculate movement from start
    movement_x = current_x - self.start_position[0]
    movement_y = current_y - self.start_position[1]
    movement_distance = math.sqrt(movement_x**2 + movement_y**2)
    
    # Expected movement
    expected_movement_x = self.expected_position[0] - self.start_position[0]
    expected_movement_y = self.expected_position[1] - self.start_position[1]
    expected_movement_distance = math.sqrt(expected_movement_x**2 + expected_movement_y**2)
    
    # Orientation change
    orientation_change = current_theta - self.start_orientation
    while orientation_change > math.pi:
      orientation_change -= 2 * math.pi
    while orientation_change < -math.pi:
      orientation_change += 2 * math.pi
    
    expected_orientation_change = self.expected_orientation - self.start_orientation
    while expected_orientation_change > math.pi:
      expected_orientation_change -= 2 * math.pi
    while expected_orientation_change < -math.pi:
      expected_orientation_change += 2 * math.pi
    
    # Orientation error (handle wrap-around)
    orient_error = current_theta - self.expected_orientation
    while orient_error > math.pi:
      orient_error -= 2 * math.pi
    while orient_error < -math.pi:
      orient_error += 2 * math.pi
    orient_error_abs = abs(orient_error)
    
    self.position_errors.append(pos_error)
    self.orientation_errors.append(orient_error_abs)
    
    # Print detailed results
    print("\n" + "="*70)
    print("CALIBRATION RESULTS")
    print("="*70)
    print(f"Action #{self.action_count}")
    print()
    print("START POSITION:")
    print(f"  Position: ({self.start_position[0]:.6f}, {self.start_position[1]:.6f}) m")
    print(f"  Orientation: {math.degrees(self.start_orientation):.3f}°")
    print()
    print("EXPECTED FINAL STATE:")
    print(f"  Position: ({self.expected_position[0]:.6f}, {self.expected_position[1]:.6f}) m")
    print(f"  Orientation: {math.degrees(self.expected_orientation):.3f}°")
    print(f"  Expected movement: {expected_movement_distance*1000:.2f} mm")
    print(f"  Expected orientation change: {math.degrees(expected_orientation_change):.3f}°")
    print()
    print("ACTUAL FINAL STATE:")
    print(f"  Position: ({current_x:.6f}, {current_y:.6f}) m")
    print(f"  Orientation: {math.degrees(current_theta):.3f}°")
    print(f"  Actual movement: {movement_distance*1000:.2f} mm")
    print(f"  Actual orientation change: {math.degrees(orientation_change):.3f}°")
    print()
    print("ERRORS:")
    print(f"  Position error (X): {pos_error_x*1000:.2f} mm")
    print(f"  Position error (Y): {pos_error_y*1000:.2f} mm")
    print(f"  Position error (total): {pos_error*1000:.2f} mm")
    print(f"  Movement distance error: {(movement_distance - expected_movement_distance)*1000:.2f} mm")
    print(f"  Orientation error: {math.degrees(orient_error_abs):.3f}°")
    print(f"  Orientation change error: {math.degrees(abs(orientation_change - expected_orientation_change)):.3f}°")
    print()
    
    # Analyze scan data if available
    if self.current_scan is not None:
      self.analyze_scan_data()
    
    if len(self.position_errors) > 1:
      avg_pos_error = sum(self.position_errors) / len(self.position_errors)
      avg_orient_error = sum(self.orientation_errors) / len(self.orientation_errors)
      print("STATISTICS (all actions):")
      print(f"  Average position error:   {avg_pos_error*1000:.2f} mm")
      print(f"  Average orientation error: {math.degrees(avg_orient_error):.3f}°")
      print(f"  Total actions: {len(self.position_errors)}")
      print()
    
    print("="*70 + "\n")
    
    # Also log to ROS logger
    self.get_logger().info(f'Calibration results - Position error: {pos_error*1000:.2f} mm, Orientation error: {math.degrees(orient_error_abs):.3f}°')
  
  def analyze_scan_data(self):
    """Analyze lidar scan data for calibration insights"""
    if self.current_scan is None:
      return
    
    scan = self.current_scan
    ranges = scan.ranges
    
    # Filter out invalid readings (inf, nan, or out of range)
    valid_ranges = [r for r in ranges if r != float('inf') and not math.isnan(r) and 
                    scan.range_min <= r <= scan.range_max]
    
    if len(valid_ranges) == 0:
      print("SCAN ANALYSIS: No valid range readings")
      return
    
    min_range = min(valid_ranges)
    max_range = max(valid_ranges)
    avg_range = sum(valid_ranges) / len(valid_ranges)
    
    # Find the angle with minimum range (closest obstacle)
    min_range_idx = ranges.index(min_range)
    angle_min = scan.angle_min + min_range_idx * scan.angle_increment
    
    # Find the angle with maximum range (furthest open space)
    max_range_idx = ranges.index(max_range)
    angle_max = scan.angle_min + max_range_idx * scan.angle_increment
    
    print("SCAN ANALYSIS:")
    print(f"  Valid readings: {len(valid_ranges)}/{len(ranges)}")
    print(f"  Min range: {min_range:.3f} m at {math.degrees(angle_min):.1f}°")
    print(f"  Max range: {max_range:.3f} m at {math.degrees(angle_max):.1f}°")
    print(f"  Average range: {avg_range:.3f} m")
    
    # Check if we're close to expected position by looking at scan consistency
    # If we're in a known location, the scan pattern should be consistent
    if self.start_position is not None:
      # Calculate distance from start
      distance_from_start = math.sqrt(
        (self.current_odom.pose.pose.position.x - self.start_position[0])**2 +
        (self.current_odom.pose.pose.position.y - self.start_position[1])**2
      )
      print(f"  Distance from start position: {distance_from_start*1000:.2f} mm")
  
  def reset_statistics(self):
    """Reset calibration statistics"""
    self.position_errors = []
    self.orientation_errors = []
    self.action_count = 0
    self.get_logger().info('Statistics reset')
  
  def print_calibration_values(self):
    """Print current calibration parameter values"""
    print("\n" + "="*60)
    print("CURRENT CALIBRATION VALUES")
    print("="*60)
    print(f"Forward distance:     {self.forward_distance:.4f} m (adjust: {self.forward_distance_adjust:+.4f} m)")
    print(f"Turn angle:           {math.degrees(self.turn_angle):.2f}° (adjust: {math.degrees(self.turn_angle_adjust):+.2f}°)")
    print(f"Linear speed:         {self.linear_speed:.4f} m/s (adjust: {self.linear_speed_adjust:+.4f} m/s)")
    print(f"Angular speed:        {self.angular_speed:.4f} rad/s (adjust: {self.angular_speed_adjust:+.4f} rad/s)")
    print("="*60 + "\n")
  
  def scan_callback(self, msg: LaserScan):
    """Callback for lidar scan messages"""
    self.current_scan = msg
    # Could use scan data for additional calibration checks
  
  def odom_callback(self, msg: Odometry):
    """Callback for odometry messages"""
    self.current_odom = msg

  def destroy_node(self):
    """Cleanup on node destruction"""
    self._stop_keyboard.set()
    # Send stop command before shutdown
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_vel_pub.publish(twist)
    return super().destroy_node()

def main(args=None):
  rclpy.init(args=args)
  node = OdomCalibrationNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()

