#!/usr/bin/env python3

"""
QoS Relay Node
Receives odometry data with RELIABLE QoS and republishes with BEST_EFFORT QoS
to make it compatible with cartographer_node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
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
        
        # Subscribe with RELIABLE QoS
        self.subscription = self.create_subscription(
            Odometry,
            'odom_uros',
            self.odom_callback,
            reliable_qos
        )
        
        # Publish with BEST_EFFORT QoS
        self.publisher = self.create_publisher(
            Odometry,
            'odom',
            best_effort_qos
        )
        
        self.get_logger().info(f'QoS Relay started: {self.namespace}/odom_uros (RELIABLE) -> {self.namespace}/odom (BEST_EFFORT)')
    
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
        self.publisher.publish(relay_msg)

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
