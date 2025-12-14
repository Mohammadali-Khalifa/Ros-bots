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
        self.declare_parameter('focal_px', 0.0)              # set from calibration
        self.declare_parameter('target_distance_m', 0.15)    # 15 cm

        # --- PID gains ---
        self.declare_parameter('kp_ang', 1.5)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.05)

        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.05)

        # --- Command limits ---
        self.declare_parameter('max_lin', 0.4)
        self.declare_parameter('max_ang', 1.2)

        # --- Simple “weak motor” helpers ---
        self.declare_parameter('deadband', 0.06)     # ignore small center noise
        self.declare_parameter('min_turn', 0.25)     # minimum turning effort
        self.declare_parameter('turn_first', 0.25)   # if ball far from center, stop forward
        self.declare_parameter('turn_bias', 0.0)     # + turns left, - turns right

        # --- Read params ---
        self.frame_width = float(self.get_parameter('frame_width').value)

        self.ball_diameter_m = float(self.get_parameter('ball_diameter_m').value)
        self.focal_px = float(self.get_parameter('focal_px').value)
        self.target_distance_m = float(self.get_parameter('target_distance_m').value)

        self.deadband = float(self.get_parameter('deadband').value)
        self.min_turn = float(self.get_parameter('min_turn').value)
        self.turn_first = float(self.get_parameter('turn_first').value)
        self.turn_bias = float(self.get_parameter('turn_bias').value)

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

        self.pid_lin = PID(
            float(self.get_parameter('kp_lin').value),
            float(self.get_parameter('ki_lin').value),
            float(self.get_parameter('kd_lin').value),
            0.0,       # don’t drive backwards
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

        self.last_msg_time = None
        self.last_twist = Twist()
        self.create_timer(0.05, self.tick)

        self.get_logger().info('FollowBallController started.')
        if self.focal_px <= 0.0:
            self.get_logger().warn("focal_px <= 0, distance control will not work (lin_x forced to 0).")

    def tick(self):
        now = self.get_clock().now()
        if self.last_msg_time is None:
            return

        dt_since_last = (now - self.last_msg_time).nanoseconds * 1e-9
        if dt_since_last > 0.3:
            twist = Twist()
            twist.angular.z = self.min_turn
            self.pub_cmd.publish(twist)
            return

        # keep publishing the most recent command (helps if image_info rate is low/uneven)
        self.pub_cmd.publish(self.last_twist)
    
    def on_measurement(self, msg: Int32MultiArray):
        now = self.get_clock().now()
        self.last_msg_time = now
        
        # safety
        if len(msg.data) < 2:
            self.pub_cmd.publish(Twist())
            return

        center = float(msg.data[0])
        width = float(msg.data[1])

        # If ball not detected -> stop + reset
        if width <= 0.0 or center < 0.0:
            self.pid_ang.reset()
            self.pid_lin.reset()
            self.last_twist = Twist()
            self.pub_cmd.publish(Twist())
            return

        # --- Angular control (center the ball) ---
        half_w = self.frame_width / 2.0
        if half_w <= 0.0:
            self.pub_cmd.publish(Twist())
            return

        # normalized error: -1..+1
        err_center = (center - half_w) / half_w

        # deadband to prevent twitch
        if abs(err_center) < self.deadband:
            ang_z = 0.0
        else:
            ang_z = self.pid_ang.update(-err_center, now)

            # minimum turn boost so weak wheel still turns robot
            if ang_z > 0.0:
                ang_z = max(ang_z, self.min_turn)
            else:
                ang_z = min(ang_z, -self.min_turn)

        # add bias to fight drift (if needed)
        ang_z = ang_z + self.turn_bias

        # --- Linear control (keep 15cm distance) ---
        if self.focal_px <= 0.0:
            lin_x = 0.0
        else:
            Z = (self.focal_px * self.ball_diameter_m) / max(width, 1.0)
            err_dist = Z - self.target_distance_m   # positive => too far => go forward
            lin_x = self.pid_lin.update(err_dist, now)

        # turn-first: if ball far from center, don’t drive forward yet
        if abs(err_center) > self.turn_first:
            lin_x = 0.0

        twist = Twist()
        twist.linear.x = lin_x
        twist.angular.z = ang_z
        self.last_twist = twist
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
