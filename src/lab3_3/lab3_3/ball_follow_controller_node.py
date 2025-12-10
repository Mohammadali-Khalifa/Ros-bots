from math import isnan

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class PID:
    def __init__(self, kp, ki, kd, u_min, u_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.u_min = u_min
        self.u_max = u_max
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def update(self, error, now):
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return self.kp * error

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        de = (error - self.prev_error) / dt
        u = self.kp * error + self.ki * self.integral + self.kd * de

        if u > self.u_max:
            u = self.u_max
        if u < self.u_min:
            u = self.u_min

        self.prev_time = now
        self.prev_error = error
        return u


class BallFollowControllerNode(Node):
    def __init__(self):
        super().__init__('ball_follow_controller_node')

        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('desired_width', 120.0)

        self.declare_parameter('kp_ang', 0.003)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.0005)

        self.declare_parameter('kp_lin', 0.002)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.0001)

        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)

        self.image_width = float(self.get_parameter('image_width').value)
        self.desired_width = float(self.get_parameter('desired_width').value)

        max_lin = float(self.get_parameter('max_linear_speed').value)
        max_ang = float(self.get_parameter('max_angular_speed').value)

        self.pid_ang = PID(
            float(self.get_parameter('kp_ang').value),
            float(self.get_parameter('ki_ang').value),
            float(self.get_parameter('kd_ang').value),
            -max_ang,
            max_ang,
        )

        self.pid_lin = PID(
            float(self.get_parameter('kp_lin').value),
            float(self.get_parameter('ki_lin').value),
            float(self.get_parameter('kd_lin').value),
            0.0,
            max_lin,
        )

        self.create_subscription(
            Float32MultiArray,
            'ball_measurements',
            self.on_measurement,
            10,
        )
        self.pub_cmd = self.create_publisher(Twist, 'auto_cmd_vel', 10)
        self.get_logger().info('ball_follow_controller_node started')

    def on_measurement(self, msg):
        now = self.get_clock().now()
        if len(msg.data) < 2:
            return

        center_px = float(msg.data[0])
        width_px = float(msg.data[1])

        if isnan(center_px) or width_px <= 0.0:
            self.pid_ang.reset()
            self.pid_lin.reset()
            self.pub_cmd.publish(Twist())
            return

        half_width = self.image_width / 2.0
        error_px = (center_px - half_width) / half_width
        ang_error = -error_px
        lin_error = self.desired_width - width_px

        ang_output = self.pid_ang.update(ang_error, now)
        lin_output = self.pid_lin.update(lin_error, now)

        twist = Twist()
        twist.linear.x = lin_output
        twist.angular.z = ang_output
        self.pub_cmd.publish(twist)


def main():
    rclpy.init()
    node = BallFollowControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
