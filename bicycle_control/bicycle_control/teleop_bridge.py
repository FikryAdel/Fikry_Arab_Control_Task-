"""
Teleoperation bridge node:
Subscribes to standard geometry_msgs/Twist on /cmd_vel (from teleop_twist_keyboard or joy)
and translates it to /throttle (Float32 in [-1.0, 1.0]) and /steer (Float32 in radians).

Supports two progression phases:
- Phase 1 (Milestone 3): Open-loop feedforward mapping with a safety watchdog timer.
- Phase 2 (Milestone 4): Closed-loop speed regulation using PIDLongitudinalController.
"""

import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.get_logger().info('Teleoperation Bridge Node Initialized')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)     # m/s corresponding to full 1.0 throttle
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s corresponding to full steering
        self.declare_parameter('max_steer_rad', 0.610865)  # radians (~35 degrees)
        self.declare_parameter('auto_zero_timeout', 0.5)  # seconds before zeroing commands
        self.declare_parameter('use_cruise_control', False)  # Enable in Milestone 4.2

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.auto_zero_timeout = float(self.get_parameter('auto_zero_timeout').value)
        self.use_cruise_control = bool(self.get_parameter('use_cruise_control').value)

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.current_vel = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Milestone 4.2: closed-loop cruise control setup
        self.pid = PIDLongitudinalController(kp=1.0, ki=0.2, kd=0.05, dt=0.1)
        self.state_sub = self.create_subscription(
            Odometry, '/state', self.odom_callback, 10)
        if self.use_cruise_control:
            self.get_logger().info('Cruise control ENABLED: linear.x is a target speed (m/s)')

        # Publish loop at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

    def odom_callback(self, msg: Odometry):
        """Milestone 4.2: Extracts vehicle forward speed from /state odometry."""
        self.current_vel = float(msg.twist.twist.linear.x)

    def cmd_callback(self, msg: Twist):
        """Translates Twist linear.x to throttle [-1, 1] and angular.z into steering (rad)."""
        # Target speed (used by cruise control) and open-loop throttle mapping
        self.target_vel = max(0.0, float(msg.linear.x))

        throttle = msg.linear.x / self.max_linear_vel
        self.current_throttle = float(np.clip(throttle, -1.0, 1.0))

        steer = (msg.angular.z / self.max_angular_vel) * self.max_steer_rad
        self.current_steer = float(np.clip(steer, -self.max_steer_rad, self.max_steer_rad))

        # Refresh the watchdog
        self.last_cmd_time = self.get_clock().now()

    def publish_commands(self):
        """Periodically publishes throttle and steering commands at 10 Hz."""
        # Safety watchdog
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.auto_zero_timeout:
            self.current_throttle = 0.0
            self.current_steer = 0.0
            self.target_vel = 0.0

        if self.use_cruise_control:
            # Closed loop: PID turns target speed into throttle/brake
            throttle_out = self.pid.compute(self.target_vel, self.current_vel)
        else:
            # Open loop
            throttle_out = self.current_throttle

        t_msg = Float32()
        t_msg.data = float(throttle_out)
        self.throttle_pub.publish(t_msg)

        s_msg = Float32()
        s_msg.data = float(self.current_steer)
        self.steer_pub.publish(s_msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = TeleopBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()