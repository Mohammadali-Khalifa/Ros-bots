import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class MovesSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        self.publisher_ = self.create_publisher(Twist, '/auto/cmd_vel', 10) #Changed from /turtle1/cmd_vel to /auto/cmd_vel
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.side = 0   # for sides done
        self.position = 0   # when the turtle moves states
        self.time = 0.0 # time

 #lines  7-12 set up the time varable as well as keep count for the number of sides done, the state of the turtle (side or turn), and the time travling. 
    def timer_callback(self):
        msg = Twist()
        if self.side < 4:                                       # condition to keep going until all sides are done
            if self.position == 0:
                msg.linear.x = 2.0
                self.publisher_.publish(msg)
                self.time = (self.time + self.timer_period)
                if self.time >= 2:       
                    self.position = 1
                    self.time = 0.0
            else:
                msg.angular.z = 1.570796326794896               # turns about 90 degrees
                self.publisher_.publish(msg)
                self.time = (self.time + self.timer_period)
                if self.time >= 1.0:      
                    self.position = 0
                    self.side += 1
                    self.time = 0.0  
                self.publisher_.publish(msg)  
            return
        else:
            self.side = 0                

#lines 15-35 has a exit condition to check if all sides are done. If not, then it toggles between the sides and turn where sides make the four side of the square, and turn rotates the turtle 90 degrees.
def main(args=None):
    rclpy.init(args=args)
    node = MovesSquare()               
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


