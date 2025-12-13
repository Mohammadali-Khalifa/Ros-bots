from math import isnan

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


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
            u = self.kp * error
            return max(self.u_min, min(self.u_max, u))

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        de = (error - self.prev_error) / dt

        u = self.kp * error + self.ki * self.integral + self.kd * de
        u = max(self.u_min, min(self.u_max, u))

        self.prev_time = now
        self.prev_error = error
        return u


class FollowBallControllerNode(Node):
    def __init__(self):
        super().__init__('follow_ball_controller')

        # --- Image + target settings ---
        self.declare_parameter('frame_width', 640.0)

        # --- Distance model parameters ---
        self.declare_parameter('ball_diameter_m', 0.04)      # 4 cm ball
        self.declare_parameter('focal_px', 0.0)              # MUST set from calibration
        self.declare_parameter('target_distance_m', 0.15)    # 15 cm

        # --- PID gains ---
        self.declare_parameter('kp_ang', 0.004)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.0008)

        # linear PID now works on distance error (meters)
        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.05)

        # --- Command limits ---
        self.declare_parameter('max_lin', 0.4)
        self.declare_parameter('max_ang', 1.2)

        # --- Read params ---
        self.frame_width = float(self.get_parameter('frame_width').value)

        self.ball_diameter_m = float(self.get_parameter('ball_diameter_m').value)
        self.focal_px = float(self.get_parameter('focal_px').value)
        self.target_distance_m = float(self.get_parameter('target_distance_m').value)

        max_lin = float(self.get_parameter('max_lin').value)
        max_ang = float(self.get_parameter('max_ang').value)

        # --- PID controllers ---
        self.pid_ang = PID(
            float(self.get_parameter('kp_ang').value),
            float(self.get_parameter('ki_ang').value),
            float(self.get_parameter('kd_ang').value),
            -max_ang,
            max_ang,
        )

        # don’t drive backwards
        self.pid_lin = PID(
            float(self.get_parameter('kp_lin').value),
            float(self.get_parameter('ki_lin').value),
            float(self.get_parameter('kd_lin').value),
            0.0,
            max_lin,
        )

        # --- ROS I/O ---
        self.create_subscription(
            Int32MultiArray,
            'image_info',           # [center_px, width_px]
            self.on_measurement,
            10,
        )
        self.pub_cmd = self.create_publisher(Twist, 'auto/cmd_vel', 10)

        self.get_logger().info('FollowBallController (distance-based) started.')
        if self.focal_px <= 0.0:
            self.get_logger().warn("Parameter 'focal_px' is <= 0. Set it, or distance control won't work.")

    def on_measurement(self, msg: Int32MultiArray):
        now = self.get_clock().now()

        if len(msg.data) < 2:
            self.pub_cmd.publish(Twist())
            return

        center_px = float(msg.data[0])
        width_px = float(msg.data[1])

        # If ball not detected -> stop + reset
        if width_px <= 0.0 or center_px < 0.0:
            self.pid_ang.reset()
            self.pid_lin.reset()
            self.pub_cmd.publish(Twist())
            return
            
        # --- Angular control (keep centered) ---
        half_w = self.frame_width / 2.0
        if half_w <= 0.0:
            self.pub_cmd.publish(Twist())
            return

        error_px = (center_px - half_w) / half_w   # -1..+1
        ang_error = -error_px                      # positive -> turn left
        ang_z = self.pid_ang.update(ang_error, now)

        # --- Distance estimate + linear control ---
        # Z = (f * D) / w
        denom = max(width_px, 1.0)
        if self.focal_px <= 0.0:
            # if focal isn't set, safest behavior is: no forward motion
            lin_x = 0.0
        else:
            Z = (self.focal_px * self.ball_diameter_m) / denom
            err_dist = Z - self.target_distance_m   # positive => too far => go forward
            lin_x = self.pid_lin.update(err_dist, now)

        twist = Twist()
        twist.linear.x = lin_x
        twist.angular.z = ang_z
        self.pub_cmd.publish(twist)


def main():
    rclpy.init()
    node = FollowBallControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
