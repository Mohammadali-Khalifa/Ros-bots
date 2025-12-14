import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0.0
        self.prev_e = 0.0
    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0
    def step(self, e, dt):
        self.i += e * dt
        d = (e - self.prev_e) / dt if dt > 0.0 else 0.0
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * d
class FollowBallController(Node):
    def __init__(self):
        super().__init__('follow_ball_controller')
        self.frame_width = 640.0
        self.target_width = 220.0  
        self.pid_turn = PID(kp=2.0, ki=0.0, kd=0.20)
        self.pid_fwd = PID(kp=0.006, ki=0.0, kd=0.001)
        self.max_lin = 0.50
        self.max_ang = 2.00
        self.min_forward = 0.18   
        self.min_turn = 0.40      
        self.deadband = 0.05      # ignore tiny center noise
        self.turn_first = 0.25    # if ball far from center, don't drive forward
        self.turn_bias = 0.0      # try -0.15 if it always rotates CCW, +0.15 if CW
        self.prev_time = self.get_clock().now()
        self.create_subscription(Int32MultiArray, 'image_info', self.cb, 10)
        self.pub = self.create_publisher(Twist, 'auto/cmd_vel', 10)
        self.get_logger().info("Two-PID FollowBallController started.")
    def cb(self, msg: Int32MultiArray):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            dt = 0.02
        if len(msg.data) < 2:
            self.pub.publish(Twist())
            return
        center = float(msg.data[0])
        width = float(msg.data[1])
        if width <= 0.0 or center <= 0.0:
            self.pid_turn.reset()
            self.pid_fwd.reset()
            self.pub.publish(Twist())
            return
        half_w = self.frame_width / 2.0
        # normalized center error: left=-1 .. right=+1
        err_center = (center - half_w) / half_w
        if abs(err_center) < self.deadband:
            ang_z = 0.0
            self.pid_turn.reset()   # keeps it from twitching at center
        else:
            ang_z = self.pid_turn.step(-err_center, dt)
            if ang_z > 0.0:
                ang_z = max(ang_z, self.min_turn)
            else:
                ang_z = min(ang_z, -self.min_turn)
        ang_z += self.turn_bias
        ang_z = max(-self.max_ang, min(self.max_ang, ang_z))
        # width error: positive => too far => move forward
        err_width = self.target_width - width
        lin_x = self.pid_fwd.step(err_width, dt)
        if lin_x < 0.0:
            lin_x = 0.0
        if abs(err_center) > self.turn_first:
            lin_x = 0.0
            self.pid_fwd.reset()
        if lin_x > 0.0 and lin_x < self.min_forward:
            lin_x = self.min_forward
        lin_x = max(0.0, min(self.max_lin, lin_x))
        twist = Twist()
        twist.linear.x = lin_x
        twist.angular.z = ang_z
        self.pub.publish(twist)
