import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose 
from odometry_helper_msg.msg import DistWheel 
import math 

class RobotMove(Node):
    def __init__(self):
        super().__init__('robot_move')

        self.X = 0 #initiates X at 0
        self.Y = 0 #initiates Y at 0
        self.T = 0 #initiates Theta at 0
 
        self.subscription = self.create_subscription(DistWheel, 'dist_wheel', self.callback, 10)
        self.publisher_ = self.create_publisher(Pose, 'pose', 10)  
        #lines 15 and 16 set the subcriber and pubblisher so it reads the movements, does the math and publishes it to get graphed

    def callback(self, msg): # call back every second per the the DistWheel timmer
        left_wheel = msg.dist_wheel_left 
        right_wheel = msg.dist_wheel_right
        # lines 20 and 21 saves the left and right wheel movements in a varable

        Delta_S = (left_wheel + right_wheel) / 2.0
        Delta_T = (right_wheel - left_wheel) / (0.1) # 0.1 is from 2*L where L is 0.05 ->"2L=0.1m (so L=0.05m)" 

        Middle = self.T + (Delta_T / 2.0)
        Delta_X = Delta_S * math.cos(Middle)
        Delta_Y = Delta_S * math.sin(Middle)

        self.X += Delta_X
        self.Y += Delta_Y
        self.T += Delta_T

        self.T = math.atan2(math.sin(self.T), math.cos(self.T))
        # lines 24-29 does the math shown in slides 96-105 of Cordinate Transormations

        pose_msg = Pose()
        pose_msg.x = self.X
        pose_msg.y = self.Y
        pose_msg.theta = self.T
        self.publisher_.publish(pose_msg)

        #lines 38 - 41 publish the new data and then the cycle repeats again
def main(args=None):
    rclpy.init(args=args)
    node = RobotMove()               
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
