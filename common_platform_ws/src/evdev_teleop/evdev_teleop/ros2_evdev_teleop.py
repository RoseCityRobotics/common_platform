#!/usr/bin/env python3
import threading
import time
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

KEY_DOWN = 1
KEY_UP = 0
KEY_HOLD = 2

# WASD + QE (yaw), arrows also work
KEYMAP = {
    ecodes.KEY_W: ('lin', +1),
    ecodes.KEY_S: ('lin', -1),
    ecodes.KEY_A: ('ang', +1),
    ecodes.KEY_D: ('ang', -1),
    ecodes.KEY_UP: ('lin', +1),
    ecodes.KEY_DOWN: ('lin', -1),
    ecodes.KEY_LEFT: ('ang', +1),
    ecodes.KEY_RIGHT: ('ang', -1),
    ecodes.KEY_Q: ('ang', +1),
    ecodes.KEY_E: ('ang', -1),
    # Additional mappings
    ecodes.KEY_SPACE: ('stop', 0),   # immediate stop
    ecodes.KEY_COMMA: ('lin', -1),   # reverse alias
}

class EvdevTeleop(Node):
    def __init__(self):
        super().__init__('evdev_teleop')
        self.declare_parameter('device_path', '/dev/input/event0')
        self.declare_parameter('topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 0.3)   # m/s
        self.declare_parameter('angular_speed', 1.2)  # rad/s
        self.declare_parameter('repeat_hz', 20.0)     # publish rate while held
        self.declare_parameter('grab_device', True)   # exclusive grab
        self.declare_parameter('debug', True)         # verbose logging

        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.v_lin = float(self.get_parameter('linear_speed').value)
        self.v_ang = float(self.get_parameter('angular_speed').value)
        self.rate = float(self.get_parameter('repeat_hz').value)
        self.grab = bool(self.get_parameter('grab_device').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.pub = self.create_publisher(Twist, self.topic, 10)

        self.lin_dir = 0    # -1,0,+1
        self.ang_dir = 0

        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        # Show fully-resolved names to help with namespace debugging
        self.get_logger().info(
            f'evdev teleop from {self.device_path} → {self.pub.topic} (debug={self.debug}) node={self.get_fully_qualified_name()}'
        )

    def _loop(self):
        try:
            dev = InputDevice(self.device_path)
        except Exception as e:
            self.get_logger().error(f'Cannot open {self.device_path}: {e}')
            return
        try:
            if self.grab:
                dev.grab()  # prevent events from going to the console/X
        except Exception as e:
            self.get_logger().warn(f'Could not grab device exclusively: {e}')

        t_last = 0.0
        period = 1.0 / max(self.rate, 1.0)

        while not self._stop.is_set():
            # Non-blocking read; fall back to timed publish
            for event in dev.read_loop():
                if event.type == ecodes.EV_KEY:
                    keyevent = event.value  # 0 up, 1 down, 2 hold
                    code = event.code
                    if code in KEYMAP:
                        kind, val = KEYMAP[code]
                        if keyevent == KEY_DOWN or keyevent == KEY_HOLD:
                            if kind == 'lin':
                                self.lin_dir = val
                            elif kind == 'ang':
                                self.ang_dir = val
                            elif kind == 'stop':
                                # Immediate stop: zero both axes
                                self.lin_dir = 0
                                self.ang_dir = 0
                            if self.debug:
                                state = 'down' if keyevent == KEY_DOWN else 'hold'
                                try:
                                    key_name = ecodes.KEY[code]
                                except Exception:
                                    key_name = str(code)
                                if kind == 'stop':
                                    self.get_logger().info(f'key {key_name} {state}: STOP → lin=0 ang=0')
                                else:
                                    self.get_logger().info(
                                        f'key {key_name} {state}: {kind}={val} → lin={self.lin_dir} ang={self.ang_dir}'
                                    )
                        elif keyevent == KEY_UP:
                            # On key-up, clear only that axis if matching
                            if kind == 'lin' and self.lin_dir == val:
                                self.lin_dir = 0
                            if kind == 'ang' and self.ang_dir == val:
                                self.ang_dir = 0
                            if self.debug:
                                try:
                                    key_name = ecodes.KEY[code]
                                except Exception:
                                    key_name = str(code)
                                self.get_logger().info(
                                    f'key {key_name} up → lin={self.lin_dir} ang={self.ang_dir}'
                                )

                now = time.time()
                if now - t_last >= period:
                    t_last = now
                    self._publish()

                if self._stop.is_set():
                    break

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.v_lin * self.lin_dir
        msg.angular.z = self.v_ang * self.ang_dir
        self.pub.publish(msg)
        if self.debug:
            self.get_logger().info(
                f'publish cmd_vel: linear.x={msg.linear.x:.3f} angular.z={msg.angular.z:.3f}'
            )

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()

def main():
    rclpy.init()
    node = EvdevTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

