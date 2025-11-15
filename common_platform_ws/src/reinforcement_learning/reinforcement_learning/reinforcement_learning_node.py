#!/usr/bin/env python3

"""
Reinforcement Learning Node

This node loads a trained Q-table model and uses it to control the robot.
It subscribes to lidar scan and odom topics, discretizes the state,
selects actions using the Q-table, and publishes cmd_vel commands for
discrete movements matching the calibration actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import threading
import pickle
from collections import defaultdict
from enum import Enum
from typing import Union
import numpy as np
import os

class ActionType(Enum):
  FORWARD = 0  # Action 0: move forward
  LEFT = 1    # Action 1: turn left 90° then forward
  RIGHT = 2   # Action 2: turn right 90° then forward
  BACK = 3    # Action 3: turn 180° then forward

class ReinforcementLearningNode(Node):
  def __init__(self):
    super().__init__('reinforcement_learning_node')
    
    # Parameters
    self.declare_parameter('model_path', 'q-tabular.pkl')
    self.declare_parameter('forward_distance', 0.15)  # 15 cm default
    self.declare_parameter('turn_angle', math.pi / 2.0)  # 90 degrees
    self.declare_parameter('linear_speed', 0.3)  # m/s
    self.declare_parameter('angular_speed', 0.5)  # rad/s
    self.declare_parameter('scan_topic', 'scan')
    self.declare_parameter('odom_topic', 'odom')
    self.declare_parameter('cmd_vel_topic', 'cmd_vel')
    self.declare_parameter('namespace', '')
    self.declare_parameter('action_rate', 1.0)  # Actions per second
    self.declare_parameter('cell_size', 0.10)  # State discretization cell size
    self.declare_parameter('collision_threshold', 0.05)  # Minimum clearance in meters (similar to RAY_STOP_TOL)
    self.declare_parameter('forward_scan_angle_range', 30.0)  # Degrees to check on each side of forward direction
    
    # Get parameters
    model_path = self.get_parameter('model_path').get_parameter_value().string_value
    self.forward_distance = self.get_parameter('forward_distance').value
    self.turn_angle = self.get_parameter('turn_angle').value
    self.linear_speed = self.get_parameter('linear_speed').value
    self.angular_speed = self.get_parameter('angular_speed').value
    scan_topic = self.get_parameter('scan_topic').value
    odom_topic = self.get_parameter('odom_topic').value
    cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
    namespace = self.get_parameter('namespace').value
    self.action_rate = self.get_parameter('action_rate').value
    self.cell_size = self.get_parameter('cell_size').value
    self.collision_threshold = self.get_parameter('collision_threshold').value
    self.forward_scan_angle_range = math.radians(self.get_parameter('forward_scan_angle_range').value)
    
    # Add namespace if provided
    if namespace:
      scan_topic = f'{namespace}/{scan_topic}'
      odom_topic = f'{namespace}/{odom_topic}'
      cmd_vel_topic = f'{namespace}/{cmd_vel_topic}'
    
    # Load Q-table model
    if not os.path.isabs(model_path):
      # If relative path, try to find it relative to package or current directory
      package_share = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
      possible_paths = [
        os.path.join(package_share, model_path),
        os.path.join(os.path.dirname(__file__), '..', '..', model_path),
        model_path
      ]
      for path in possible_paths:
        if os.path.exists(path):
          model_path = path
          break
    
    self.get_logger().info(f'Loading Q-table model from: {model_path}')
    try:
      model_data = self.load_model(model_path)
      q_table_dict = model_data.get("q_table", {})
      # Convert to defaultdict for easier access
      self.q_table = defaultdict(lambda: np.zeros(4, dtype=np.float32))
      self.q_table.update(q_table_dict)
      self.cell_size = model_data.get("cell_size", self.cell_size)
      self.get_logger().info(f'Model loaded successfully!')
      self.get_logger().info(f'  Cell size: {self.cell_size}')
      self.get_logger().info(f'  Q-table entries: {len(q_table_dict)}')
    except Exception as e:
      self.get_logger().error(f'Failed to load model: {e}')
      raise
    
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
    self.is_executing_action = False
    self.action_lock = threading.Lock()
    self.collision_detected = False  # Track if collision was detected during current action
    self.stop_command_end_time = None  # Time until which we should keep publishing stop commands
    
    # Command execution
    self.cmd_vel_queue = []
    self.cmd_start_time = None
    self.current_cmd_index = 0
    
    # Timer for publishing cmd_vel during actions
    self.cmd_timer = self.create_timer(0.1, self.cmd_vel_timer_callback)
    
    # Timer for action selection (rate-limited)
    action_period = 1.0 / self.action_rate if self.action_rate > 0 else 1.0
    self.action_timer = self.create_timer(action_period, self.action_timer_callback)
    
    self.get_logger().info('Reinforcement learning node started')
    self.get_logger().info(f'Subscribed to: {scan_topic}, {odom_topic}')
    self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
    self.get_logger().info(f'Action rate: {self.action_rate} Hz')
    self.get_logger().info(f'Forward distance: {self.forward_distance*100:.1f} cm, Linear speed: {self.linear_speed:.3f} m/s')
    self.get_logger().info(f'Cell size: {self.cell_size:.3f} m (from model: {model_data.get("cell_size", "N/A")})')
    self.get_logger().info(f'Collision threshold: {self.collision_threshold:.3f} m, Forward scan angle range: {math.degrees(self.forward_scan_angle_range):.1f}°')
  
  def load_model(self, model_path: str) -> dict:
    """Load the trained Q-table model from a pickle file."""
    with open(model_path, "rb") as f:
      data = pickle.load(f)
    return data
  
  def discretize_state(self, state: np.ndarray) -> tuple[int, int, int]:
    """
    Map continuous (x, y, yaw) to coarse discrete bins. Matches tabular Q-learning defaults.
    """
    x, y, yaw = float(state[0]), float(state[1]), float(state[2])
    i = int(round(x / self.cell_size))
    j = int(round(y / self.cell_size))
    yaw = yaw % (2.0 * math.pi)
    heading_bin = int(round(yaw / (math.pi / 2.0))) % 4
    return i, j, heading_bin
  
  def check_collision(self, yaw: float = None) -> bool:
    """
    Check for obstacles in the forward direction using lidar scan.
    Similar to MazeEnv._check_wall_contact() but using scan data.
    
    Args:
      yaw: Robot orientation in radians (unused, kept for API compatibility).
          Scan is already in robot frame, so yaw is not needed.
    
    Returns:
      True if obstacle detected within collision_threshold, False otherwise.
    """
    if self.current_scan is None:
      # No scan data available, assume no collision
      self.get_logger().debug('No scan data available for collision check')
      return False
    
    scan = self.current_scan
    ranges = list(scan.ranges)
    
    # Calculate angle for each range reading
    # For RPLidar, the scan coordinate frame depends on mounting
    # Typically: angle_min to angle_max, with forward at some angle
    # Common: angle_min = -π, angle_max = π, forward at angle = 0
    # Or: angle_min = 0, angle_max = 2π, forward at angle = 0
    forward_clearances = []
    forward_angle_tolerance = self.forward_scan_angle_range  # ±30 degrees default
    
    # Find the index corresponding to forward direction (0 radians)
    # Forward is typically at angle = 0, but we need to find which index that is
    forward_index = None
    if scan.angle_min <= 0 <= scan.angle_max:
      # Forward (0 radians) is within the scan range
      forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
      forward_index = max(0, min(forward_index, len(ranges) - 1))
    elif scan.angle_min < -math.pi and scan.angle_max > math.pi:
      # Scan wraps around, forward is at the middle
      forward_index = len(ranges) // 2
    else:
      # Assume forward is at angle = 0, find closest index
      forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment)) % len(ranges)
    
    for i, r in enumerate(ranges):
      if r == float('inf') or math.isnan(r) or r < scan.range_min or r > scan.range_max:
        continue
      
      # Calculate angle for this reading
      angle = scan.angle_min + i * scan.angle_increment
      
      # Normalize angle to [-pi, pi]
      while angle > math.pi:
        angle -= 2 * math.pi
      while angle < -math.pi:
        angle += 2 * math.pi
      
      # Calculate angle relative to forward (0 radians)
      angle_rel_forward = angle - 0.0
      # Normalize to [-pi, pi]
      while angle_rel_forward > math.pi:
        angle_rel_forward -= 2 * math.pi
      while angle_rel_forward < -math.pi:
        angle_rel_forward += 2 * math.pi
      
      # Check if within forward fan range (similar to environment's -30 to +30 degrees)
      if abs(angle_rel_forward) <= forward_angle_tolerance:
        # Calculate forward clearance (projected distance along forward direction)
        # Similar to environment's forward_clearance calculation
        angle_mag = abs(angle_rel_forward)
        projected_forward = r * math.cos(angle_mag)
        # Account for robot radius (approximate 0.08m, matching ROBOT_R in maze_env)
        robot_radius = 0.08
        origin_forward_offset = robot_radius * (1.0 - math.cos(angle_mag))
        forward_clearance = max(0.0, projected_forward - origin_forward_offset)
        forward_clearances.append(forward_clearance)
    
    if not forward_clearances:
      # No valid readings in forward direction, log scan info for debugging
      self.get_logger().warn(f'No valid forward scan readings for collision check! Scan: angle_min={math.degrees(scan.angle_min):.1f}°, angle_max={math.degrees(scan.angle_max):.1f}°, increment={math.degrees(scan.angle_increment):.3f}°, ranges={len(ranges)}')
      return False
    
    min_clearance = min(forward_clearances)
    # Round to 3 decimal places (matching environment behavior)
    min_clearance = round(min_clearance, 3)
    
    collision = min_clearance <= self.collision_threshold
    if collision:
      self.get_logger().warn(f'Collision detected! Minimum forward clearance: {min_clearance:.3f} m (threshold: {self.collision_threshold:.3f} m), forward_clearances count: {len(forward_clearances)}')
    else:
      self.get_logger().debug(f'Collision check: min_clearance={min_clearance:.3f} m (threshold={self.collision_threshold:.3f} m), forward_clearances count: {len(forward_clearances)}')
    
    return collision
  
  def greedy_action(
    self,
    state_key: tuple[int, int, int],
    action_dim: int = 4,
    ) -> int:
    """Select the best action greedily from the Q-table (no exploration)."""
    if state_key not in self.q_table:
      # If state not in Q-table, default to action 0 (forward)
      self.get_logger().warn(f'State {state_key} not in Q-table, using action 0 (forward)')
      return 0
    
    values = self.q_table[state_key]
    max_q = np.max(values)
    best_actions = np.flatnonzero(values == max_q)
    
    # Log Q-values for debugging
    action_names = ['FORWARD', 'LEFT', 'RIGHT', 'BACK']
    q_values_str = ', '.join([f'{action_names[i]}={values[i]:.3f}' for i in range(4)])
    self.get_logger().info(f'State {state_key} Q-values: {q_values_str}, max={max_q:.3f}, selected action={int(best_actions[0])} ({action_names[int(best_actions[0])]})')
    
    # If multiple best actions, pick the first one deterministically
    return int(best_actions[0])
  
  def action_timer_callback(self):
    """Timer callback to select and execute actions at the specified rate"""
    self.get_logger().debug('Action timer callback fired')
    current_time = time.time()
    
    # Check if we're still in stop command period
    if self.stop_command_end_time is not None and current_time < self.stop_command_end_time:
      # Still publishing stop commands, skip this cycle
      self.get_logger().debug(f'Still in stop command period (ends at {self.stop_command_end_time:.3f}), skipping timer callback')
      return
    
    # Use lock to atomically check if action is executing
    with self.action_lock:
      if self.is_executing_action:
        # Still executing previous action, skip this cycle
        self.get_logger().debug('Action already executing, skipping timer callback')
        return
      
      # Also check if command queue is active (double-check)
      if self.cmd_vel_queue:
        self.get_logger().debug('Command queue still active, skipping timer callback')
        return
      
      if self.current_odom is None:
        # No odometry data yet, skip
        self.get_logger().info('No odometry data available yet, skipping timer callback')
        return
      
      # Get current state from odometry
      x = self.current_odom.pose.pose.position.x
      y = self.current_odom.pose.pose.position.y
      q = self.current_odom.pose.pose.orientation
      yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
      )
      
      state = np.array([x, y, yaw], dtype=float)
      
      # Discretize state
      state_disc = self.discretize_state(state)
      
      # Select action greedily
      action = self.greedy_action(state_disc, action_dim=4)
      
      self.get_logger().info(f'State: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°), '
                            f'Discretized: {state_disc}, Action: {action}')
      
      # Mark as executing before releasing lock
      self.is_executing_action = True
    
    # Execute action outside the lock to avoid deadlock
    # (execute_action will acquire the lock again, but we've already set is_executing_action)
    self.execute_action(action)
  
  def execute_action(self, action: int):
    """Execute a discrete action (0-3) with collision checking"""
    self.get_logger().info(f'execute_action called with action={action}')
    
    if action not in (0, 1, 2, 3):
      with self.action_lock:
        self.is_executing_action = False
      self.get_logger().error(f'Invalid action: {action}, must be 0-3')
      return
    
    self.get_logger().info(f'Action {action} is valid, checking collision...')
    
    # Check for collision before executing action (if action involves forward movement)
    # For actions that involve forward movement, check collision
    # Actions 0 (FORWARD), 1 (LEFT), 2 (RIGHT), 3 (BACK) all move forward after turning
    # Only check collision if scan data is available
    if self.current_scan is not None:
      self.get_logger().info(f'Scan data available, checking collision before action {action}')
      collision = self.check_collision()
      if collision:
        with self.action_lock:
          self.is_executing_action = False
        self.get_logger().warn(f'Collision detected before action {action}, skipping action')
        return
      self.get_logger().info(f'No collision detected, proceeding with action {action}')
    else:
      self.get_logger().info(f'No scan data available, skipping collision check for action {action}')
    
    with self.action_lock:
      self.collision_detected = False
    
    # Map action to ActionType
    action_type = ActionType(action)
    
    self.get_logger().info(f'Executing action {action} ({action_type.name})')
    
    # Generate command sequence
    commands = self.generate_action_commands(
      action_type, 
      self.forward_distance, 
      self.turn_angle,
      self.linear_speed,
      self.angular_speed
    )
    
    self.get_logger().info(f'Generated {len(commands)} commands for action {action}')
    
    # Execute commands
    self.execute_command_sequence(commands)
    self.get_logger().info(f'Command sequence started for action {action}')
  
  def generate_action_commands(self, action_type: ActionType, forward_dist: float, 
                               turn_angle: float, lin_speed: float, ang_speed: float):
    """Generate sequence of cmd_vel commands for an action"""
    commands = []
    
    if action_type == ActionType.FORWARD:
      # Just move forward
      duration = forward_dist / lin_speed
      self.get_logger().info(f'FORWARD action: distance={forward_dist*100:.1f}cm, speed={lin_speed:.3f}m/s, duration={duration:.3f}s')
      commands.append(('forward', duration, lin_speed, 0.0))
    elif action_type == ActionType.LEFT:
      # Turn left, then forward
      turn_duration = turn_angle / ang_speed
      commands.append(('turn', turn_duration, 0.0, ang_speed))
      forward_duration = forward_dist / lin_speed
      self.get_logger().info(f'LEFT action: turn={math.degrees(turn_angle):.1f}°, forward={forward_dist*100:.1f}cm, speed={lin_speed:.3f}m/s, forward_duration={forward_duration:.3f}s')
      commands.append(('forward', forward_duration, lin_speed, 0.0))
    elif action_type == ActionType.RIGHT:
      # Turn right, then forward
      turn_duration = turn_angle / ang_speed
      commands.append(('turn', turn_duration, 0.0, -ang_speed))
      forward_duration = forward_dist / lin_speed
      self.get_logger().info(f'RIGHT action: turn={math.degrees(turn_angle):.1f}°, forward={forward_dist*100:.1f}cm, speed={lin_speed:.3f}m/s, forward_duration={forward_duration:.3f}s')
      commands.append(('forward', forward_duration, lin_speed, 0.0))
    elif action_type == ActionType.BACK:
      # Turn 180, then forward
      back_turn_angle = math.pi
      turn_duration = abs(back_turn_angle) / ang_speed
      commands.append(('turn', turn_duration, 0.0, ang_speed))
      forward_duration = forward_dist / lin_speed
      self.get_logger().info(f'BACK action: turn=180°, forward={forward_dist*100:.1f}cm, speed={lin_speed:.3f}m/s, forward_duration={forward_duration:.3f}s')
      commands.append(('forward', forward_duration, lin_speed, 0.0))
    
    # Always end with stop
    commands.append(('stop', 0.5, 0.0, 0.0))
    
    return commands
  
  def execute_command_sequence(self, commands):
    """Execute a sequence of commands"""
    with self.action_lock:
      # Check if a command sequence is already running
      if self.cmd_vel_queue:
        self.get_logger().warn('Command sequence already in progress, ignoring new sequence')
        return
      
      self.cmd_vel_queue = commands.copy()
      self.cmd_start_time = time.time()
      self.current_cmd_index = 0
      self.get_logger().info(f'Command queue set with {len(commands)} commands, start_time={self.cmd_start_time:.3f}')
  
  def cmd_vel_timer_callback(self):
    """Timer callback to publish cmd_vel during action execution with collision monitoring"""
    current_time = time.time()
    
    # If we're in stop command mode (after collision), keep publishing stop commands
    if self.stop_command_end_time is not None:
      if current_time < self.stop_command_end_time:
        # Continue publishing stop command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        return
      else:
        # Stop command period complete, clear the flag
        self.stop_command_end_time = None
    
    if not self.cmd_vel_queue:
      return
    
    elapsed = current_time - self.cmd_start_time
    
    # Find which command we should be executing
    cumulative_time = 0.0
    active_cmd = None
    active_cmd_type = None
    
    for i, (cmd_type, duration, lin_vel, ang_vel) in enumerate(self.cmd_vel_queue):
      if elapsed < cumulative_time + duration:
        active_cmd = (cmd_type, lin_vel, ang_vel)
        active_cmd_type = cmd_type
        self.current_cmd_index = i
        break
      cumulative_time += duration
    
    if active_cmd:
      # Check for collision during forward movement (similar to environment's _drive_forward_distance)
      if active_cmd_type == 'forward' and not self.collision_detected:
        if self.check_collision():
          self.collision_detected = True
          self.get_logger().warn('Collision detected during forward movement, stopping')
          # Stop immediately and clear queue
          twist = Twist()
          twist.linear.x = 0.0
          twist.angular.z = 0.0
          # Publish stop command once
          self.cmd_vel_pub.publish(twist)
          self.cmd_vel_queue = []
          # Clear stop command period (no need to keep publishing)
          self.stop_command_end_time = None
          with self.action_lock:
            self.is_executing_action = False
          return
      
      # Publish active command (only if no collision detected)
      if not self.collision_detected:
        twist = Twist()
        twist.linear.x = active_cmd[1]
        twist.angular.z = active_cmd[2]
        self.cmd_vel_pub.publish(twist)
    else:
      # All commands complete
      # Send stop command once
      twist = Twist()
      twist.linear.x = 0.0
      twist.angular.z = 0.0
      self.cmd_vel_pub.publish(twist)
      
      # Clear stop command period (no need to keep publishing)
      self.stop_command_end_time = None
      
      # Log collision status if detected
      if self.collision_detected:
        self.get_logger().warn('Action completed with collision detected')
      
      # Clear queue and mark action as complete
      self.cmd_vel_queue = []
      with self.action_lock:
        self.is_executing_action = False
  
  def scan_callback(self, msg: LaserScan):
    """Callback for lidar scan messages"""
    self.current_scan = msg
  
  def odom_callback(self, msg: Odometry):
    """Callback for odometry messages"""
    self.current_odom = msg
  
  def destroy_node(self):
    """Cleanup on node destruction"""
    # Send stop command before shutdown
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_vel_pub.publish(twist)
    return super().destroy_node()

def main(args=None):
  rclpy.init(args=args)
  node = ReinforcementLearningNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()

