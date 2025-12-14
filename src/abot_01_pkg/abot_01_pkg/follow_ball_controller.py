import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class BallFollow(Node):
    def __init__(self):
        super().__init__('ball_follow_min')
        self.image_width = 640.0
        self.desired_width = 200.0     # Ball size
        self.k_ang = 1.6               # turn gain
        self.k_lin = 0.004             # forward/back gain
        self.max_ang = 2.0
        self.max_lin = 0.8
        self.center_deadband = 0.05    
        self.width_deadband = 30.0     # pixels for left and right
        
        self.sub = self.create_subscription( Int32MultiArray,'image_info', self.cb,10)
        self.pub = self.create_publisher(Twist, auto/cmd_vel', 10)
        self.get_logger().info('ball_follow_min started')
                                         
    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))
    def cb(self, msg: Int32MultiArray):
        t = Twist()
        if len(msg.data) < 2:
            self.pub.publish(t)
            return
        center_px = float(msg.data[0])
        width_px  = float(msg.data[1])

        if width_px <= 0.0 or center_px <= 0.0:
            self.pub.publish(t)
            return
        half = self.image_width / 2.0
        err_center = (center_px - half) / half   # -1..+1

        if abs(err_center) > self.center_deadband:
            t.angular.z = self.clamp(-self.k_ang * err_center, -self.max_ang, self.max_ang)
        else:
            t.angular.z = 0.0

        err_width = self.desired_width - width_px

        if abs(err_width) > self.width_deadband:
            # only drive forward if ball is too small (far)
            v = self.k_lin * err_width
            t.linear.x = self.clamp(v, 0.0, self.max_lin)
        else:
            t.linear.x = 0.0

        self.pub.publish(t)

def main(args=None):
    rclpy.init
    node = BallFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
