import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import time
class FollowBallController(Node):
   def __init__(self):
       super().__init__('follow_ball_controller')
       self.declare_parameter('frame_width', 640.0)
       self.declare_parameter('target_width', 120.0)
       self.declare_parameter('max_lin', 0.4)
       self.declare_parameter('max_ang', 1.2)
       self.declare_parameter('kp_ang', 0.004)
       self.declare_parameter('ki_ang', 0.0)
       self.declare_parameter('kd_ang', 0.0008)
       self.declare_parameter('kp_lin', 0.003)
       self.declare_parameter('ki_lin', 0.0)
       self.declare_parameter('kd_lin', 0.0006)
       self.frame_width = float(self.get_parameter('frame_width').value)
       self.target_width = float(self.get_parameter('target_width').value)
       self.max_lin = float(self.get_parameter('max_lin').value)
       self.max_ang = float(self.get_parameter('max_ang').value)
       self.kp_ang = float(self.get_parameter('kp_ang').value)
       self.ki_ang = float(self.get_parameter('ki_ang').value)
       self.kd_ang = float(self.get_parameter('kd_ang').value)
       self.kp_lin = float(self.get_parameter('kp_lin').value)
       self.ki_lin = float(self.get_parameter('ki_lin').value)
       self.kd_lin = float(self.get_parameter('kd_lin').value)
       self.prev_err_ang = 0.0
       self.int_err_ang = 0.0
       self.prev_err_lin = 0.0
       self.int_err_lin = 0.0
       self.prev_time = time.time()
       self.create_subscription(
           Int32MultiArray,
           'image_info',
           self.image_info_cb,
           10
       )
       self.cmd_pub = self.create_publisher(Twist, 'auto/cmd_vel', 10)
       self.get_logger().info('FollowBallController started.')
   def image_info_cb(self, msg: Int32MultiArray):
       if len(msg.data) < 2:
           self.get_logger().warn('image_info must contain [center_px, object_width_px]')
           return
       center_px = float(msg.data[0])
       width_px = float(msg.data[1])
       # Compute dt
       now = time.time()
       dt = now - self.prev_time
       if dt <= 0.0:
           dt = 1e-3
       self.prev_time = now
       # --- Angular PID (keep ball in center of image) ---
       desired_center = self.frame_width / 2.0
       err_ang = desired_center - center_px   # positive -> ball is left of center
       self.int_err_ang += err_ang * dt
       der_ang = (err_ang - self.prev_err_ang) / dt
       self.prev_err_ang = err_ang
       ang_z = (
           self.kp_ang * err_ang +
           self.ki_ang * self.int_err_ang +
           self.kd_ang * der_ang
       )
       # --- Linear PID (keep ball at desired size) ---
       err_lin = self.target_width - width_px   # positive -> ball too small (too far)
       self.int_err_lin += err_lin * dt
       der_lin = (err_lin - self.prev_err_lin) / dt
       self.prev_err_lin = err_lin
       lin_x = (
           self.kp_lin * err_lin +
           self.ki_lin * self.int_err_lin +
           self.kd_lin * der_lin
       )
       # Saturate commands
       if lin_x > self.max_lin:
           lin_x = self.max_lin
       if lin_x < 0.0:
           lin_x = 0.0   # don't drive backwards into the ball
       if ang_z > self.max_ang:
           ang_z = self.max_ang
       if ang_z < -self.max_ang:
           ang_z = -self.max_ang
       cmd = Twist()
       cmd.linear.x = lin_x
       cmd.angular.z = ang_z
       self.cmd_pub.publish(cmd)
       self.get_logger().debug(
           f'center={center_px:.1f}, width={width_px:.1f}, '
           f'cmd: lin_x={lin_x:.3f}, ang_z={ang_z:.3f}'
       )
def main(args=None):
   rclpy.init(args=args)
   node = FollowBallController()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
if __name__ == '__main__':
   main()
