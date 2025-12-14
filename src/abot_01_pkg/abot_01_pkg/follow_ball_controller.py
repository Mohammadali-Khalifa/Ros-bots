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
        self.declare_parameter('desired_width', 220.0)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.20)
        self.declare_parameter('kp_lin', 0.006)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.001)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('min_forward', 0.18)
        self.declare_parameter('min_turn', 0.40)
        self.declare_parameter('deadband', 0.05)
        self.declare_parameter('turn_first', 0.25)
        self.declare_parameter('turn_bias', 0.0)
        self.declare_parameter('width_deadband_px', 15.0)     
        self.declare_parameter('stop_turn_scale', 0.3)       

        self.image_width = float(self.get_parameter('image_width').value)
        self.desired_width = float(self.get_parameter('desired_width').value)

        max_lin = float(self.get_parameter('max_linear_speed').value)
        max_ang = float(self.get_parameter('max_angular_speed').value)

        self.min_forward = float(self.get_parameter('min_forward').value)
        self.min_turn = float(self.get_parameter('min_turn').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.turn_first = float(self.get_parameter('turn_first').value)
        self.turn_bias = float(self.get_parameter('turn_bias').value)

        self.width_deadband_px = float(self.get_parameter('width_deadband_px').value)
        self.stop_turn_scale = float(self.get_parameter('stop_turn_scale').value)

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
            Int32MultiArray,
            'image_info',            # [center_px, width_px]
            self.on_measurement,
            10,
        )

        self.pub_cmd = self.create_publisher(Twist, 'auto/cmd_vel', 10)
        self.get_logger().info('ball_follow_controller_node started')

    def on_measurement(self, msg):
        now = self.get_clock().now()
        if len(msg.data) < 2:
            return

        center_px = float(msg.data[0])
        width_px = float(msg.data[1])

        # not detected
        if width_px <= 0.0 or center_px <= 0.0 or isnan(center_px):
            self.pid_ang.reset()
            self.pid_lin.reset()
            self.pub_cmd.publish(Twist())
            return

        half_width = self.image_width / 2.0
        err_center = (center_px - half_width) / half_width   # -1..+1
        if abs(err_center) < self.deadband:
            ang_output = 0.0
            self.pid_ang.reset()
        else:
            ang_output = self.pid_ang.update(-err_center, now)

            if ang_output > 0.0:
                ang_output = max(ang_output, self.min_turn)
            else:
                ang_output = min(ang_output, -self.min_turn)

        ang_output = ang_output + self.turn_bias
        lin_error = self.desired_width - width_px
        if abs(lin_error) <= self.width_deadband_px:
            lin_output = 0.0
            self.pid_lin.reset()
            ang_output = ang_output * self.stop_turn_scale
        else:
            lin_output = self.pid_lin.update(lin_error, now)
            if abs(err_center) > self.turn_first:
                lin_output = 0.0
                self.pid_lin.reset()
            if lin_output > 0.0 and lin_output < self.min_forward:
                lin_output = self.min_forward
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
