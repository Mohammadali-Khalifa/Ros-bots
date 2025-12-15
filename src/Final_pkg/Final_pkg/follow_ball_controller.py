from math import isnan
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class BallFollow(Node):
    def __init__(self):
        super().__init__('ball_follow_simple')
        self.image_width = 640.0                  # camera image in pixels
        self.target_width = 200.0                   #width of the ball
        self.angular_gain = 1.6                # how much to turn left or right to see the ball
        self.linear_gain = 0.004                # how much to go forward to the ball
        self.center_deadband = 0.15            # for when the ball is centered
        self.width_deadband_px = 30.0           #this is for moving forward and back
        self.min_turn_speed = 0.50            # maintains steady speeds for turning
        self.min_forward_speed = 0.30            #mianitans steady speeds for going forwards and back (reduces the jerking)
        self.turn_first_threshold = 0.25         #how much the bot should be cenetred beofore it goes stright
        
        self.create_subscription(Int32MultiArray, 'image_info', self.image_callback, 10)  #subscibes to image_info to get the ball info
        self.cmd_pub = self.create_publisher(Twist, 'auto/cmd_vel', 10) #publishes to auto/cmd_vel to move the motors
        
    def image_callback(self, msg):
        cmd = Twist()
        if len(msg.data) < 2:
            self.cmd_pub.publish(cmd)
            return
            
        center_px = float(msg.data[0])  #gets the center pos of the ball in pixels
        width_px = float(msg.data[1])    #gets the width of the ball in pixels
        
        if width_px <= 0.0 or center_px <= 0.0 or isnan(center_px):
            self.cmd_pub.publish(cmd)
            return
                #lines 32-34 is for if the ball is not detetcted
        
        image_center = self.image_width / 2.0
        center_error = (center_px - image_center) / image_center
        #LINES 37 AND 38  are for making the ball at the center
        
        if abs(center_error) > self.center_deadband:
            angular_speed = -self.angular_gain * center_error
            if angular_speed > 0.0:
                angular_speed = max(angular_speed, self.min_turn_speed)
            else:
                angular_speed = min(angular_speed, -self.min_turn_speed)
            cmd.angular.z = angular_speed
            #lines 41-47 ajust the speed when its turning 
        
        width_error = self.target_width - width_px
        if abs(width_error) > self.width_deadband_px:
            linear_speed = self.linear_gain * width_error
            if abs(center_error) > self.turn_first_threshold:
                linear_speed = 0.0
            if 0.0 < linear_speed < self.min_forward_speed:
                linear_speed = self.min_forward_speed
            cmd.linear.x = linear_speed
            #lines 51 to 57 is for moving the robot forward or backwards determed by the ball width
        self.cmd_pub.publish(cmd) #publishes the  speed

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
