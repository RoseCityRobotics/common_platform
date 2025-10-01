#!/usr/bin/env python3
"""
Simple script to control data recording via ROS2 topic.

Usage:
    python3 record_control.py start    # Start recording
    python3 record_control.py stop     # Stop recording
    python3 record_control.py status   # Check recording status
"""

import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class RecordController(Node):
  def __init__(self):
    super().__init__('record_controller')
    
    # Handle namespace for topics
    ns = os.environ.get('ROS_NAMESPACE', '').strip()
    if ns:
      start_stop_topic = f'/{ns}/data_recorder/start_stop'
      status_topic = f'/{ns}/data_recorder/status'
    else:
      start_stop_topic = '/data_recorder/start_stop'
      status_topic = '/data_recorder/status'
    
    self.pub = self.create_publisher(Bool, start_stop_topic, 10)
    self.sub = self.create_subscription(Bool, status_topic, self.status_callback, 10)
    self.recording_status = None
    
  def status_callback(self, msg):
    self.recording_status = msg.data
    
  def start_recording(self):
    msg = Bool()
    msg.data = True
    self.pub.publish(msg)
    self.get_logger().info("Start recording command sent")
    
  def stop_recording(self):
    msg = Bool()
    msg.data = False
    self.pub.publish(msg)
    self.get_logger().info("Stop recording command sent")
    
  def check_status(self):
    # Wait a bit for status message and try multiple times
    for i in range(3):
      time.sleep(0.5)
      rclpy.spin_once(self, timeout_sec=0.1)
      if self.recording_status is not None:
        status = "RECORDING" if self.recording_status else "STOPPED"
        self.get_logger().info(f"Recording status: {status}")
        return
    
    self.get_logger().warn("Could not determine recording status - is the data recorder running?")

def main():
  if len(sys.argv) != 2:
    print("Usage: python3 record_control.py [start|stop|status]")
    return 1
  
  command = sys.argv[1].lower()
  
  if command not in ['start', 'stop', 'status']:
    print("Invalid command. Use: start, stop, or status")
    return 1
  
  rclpy.init()
  controller = RecordController()
  
  try:
    if command == 'start':
      controller.start_recording()
      # Give time for message to be sent and status to update
      rclpy.spin_once(controller, timeout_sec=1.0)
    elif command == 'stop':
      controller.stop_recording()
      # Give time for message to be sent and status to update
      rclpy.spin_once(controller, timeout_sec=1.0)
    elif command == 'status':
      controller.check_status()
    
  finally:
    controller.destroy_node()
    rclpy.shutdown()
  
  return 0

if __name__ == "__main__":
  sys.exit(main())
