from math import isnan
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class BallFollow(Node):
    def __init__(self):
        super().__init__('ball_follow_simple')
        self.image_width = 640.0
        self.desired_width = 200.0

        self.k_ang = 1.6
        self.k_lin = 0.004

        self.max_ang = 2.0
        self.max_lin = 0.8

        self.center_deadband = 0.05
        self.width_deadband_px = 30.0

        self.min_turn = 0.40      
        self.min_forward = 0.18   
        self.turn_first = 0.25     

        self.sub = self.create_subscription(Int32MultiArray, 'image_info', self.cb, 10)
        self.pub = self.create_publisher(Twist, 'auto/cmd_vel', 10)
        self.get_logger().info('ball_follow_simple started')

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def cb(self, msg: Int32MultiArray):
        t = Twist()

        if len(msg.data) < 2:
            self.pub.publish(t)
            return

        center_px = float(msg.data[0])
        width_px  = float(msg.data[1])

        # not detected
        if width_px <= 0.0 or center_px <= 0.0 or isnan(center_px):
            self.pub.publish(t)
            return

        # ---- angular ----
        half = self.image_width / 2.0
        err_center = (center_px - half) / half   # -1..+1

        if abs(err_center) > self.center_deadband:
            w = -self.k_ang * err_center
            w = self.clamp(w, -self.max_ang, self.max_ang)

            # enforce minimum turn effort
            if w > 0.0:
                w = max(w, self.min_turn)
            else:
                w = min(w, -self.min_turn)

            t.angular.z = w
        else:
            t.angular.z = 0.0

        # ---- linear ----
        err_width = self.desired_width - width_px

        if abs(err_width) > self.width_deadband_px:
            v = self.k_lin * err_width
            v = self.clamp(v, 0.0, self.max_lin)

            # turn-first: if way off-center, don't drive forward
            if abs(err_center) > self.turn_first:
                v = 0.0

            # enforce minimum forward effort
            if 0.0 < v < self.min_forward:
                v = self.min_forward

            t.linear.x = v
        else:
            t.linear.x = 0.0

        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = BallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
