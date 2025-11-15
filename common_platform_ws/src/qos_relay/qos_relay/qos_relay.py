#!/usr/bin/env python3

"""
QoS Relay Node
Receives odometry and IMU data with RELIABLE QoS and republishes with BEST_EFFORT QoS
to make it compatible with cartographer_node and other nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import os

class QoSRelay(Node):
    def __init__(self):
        super().__init__('qos_relay')
        
        # Get namespace from ROS_NAME environment variable
        self.namespace = os.getenv('ROS_NAME', '')
        
        # Create QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to odometry with RELIABLE QoS
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom_uros',
            self.odom_callback,
            reliable_qos
        )
        
        # Publish odometry with BEST_EFFORT QoS
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            best_effort_qos
        )
        
        # Subscribe to IMU with RELIABLE QoS
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_uros/data',
            self.imu_callback,
            reliable_qos
        )
        
        # Publish IMU with BEST_EFFORT QoS
        self.imu_publisher = self.create_publisher(
            Imu,
            'imu/data',
            best_effort_qos
        )
        
        self.get_logger().info(f'QoS Relay started:')
        self.get_logger().info(f'  {self.namespace}/odom_uros (RELIABLE) -> {self.namespace}/odom (BEST_EFFORT)')
        self.get_logger().info(f'  {self.namespace}/imu_uros/data (RELIABLE) -> {self.namespace}/imu/data (BEST_EFFORT)')
    
    def odom_callback(self, msg):
        # Create a new message to ensure correct topic type hash
        relay_msg = Odometry()
        
        # Override timestamp with current system time (fixes microcontroller time issues)
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.header.frame_id = self.namespace + '/' + msg.header.frame_id
        relay_msg.child_frame_id = self.namespace + '/' + msg.child_frame_id
        relay_msg.pose = msg.pose
        relay_msg.twist = msg.twist
        
        # Publish with BEST_EFFORT QoS
        self.odom_publisher.publish(relay_msg)
    
    def imu_callback(self, msg):
        # Create a new message to ensure correct topic type hash
        relay_msg = Imu()
        
        # Override timestamp with current system time (fixes microcontroller time issues)
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.header.frame_id = self.namespace + '/' + msg.header.frame_id
        
        # Copy IMU data
        relay_msg.orientation = msg.orientation
        relay_msg.orientation_covariance = msg.orientation_covariance
        relay_msg.angular_velocity = msg.angular_velocity
        relay_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        relay_msg.linear_acceleration = msg.linear_acceleration
        relay_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # Publish with BEST_EFFORT QoS
        self.imu_publisher.publish(relay_msg)

def main(args=None):
    rclpy.init(args=args)
    
    relay = QoSRelay()
    
    try:
        rclpy.spin(relay)
    except KeyboardInterrupt:
        pass
    finally:
        relay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
