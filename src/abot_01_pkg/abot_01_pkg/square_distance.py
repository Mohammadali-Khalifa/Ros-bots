import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose

class SquareDistance(Node):
    def __init__(self):
        super().__init__('square_distance')
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.distance, 10)
        self.publisher_ = self.create_publisher(Float32, 'turtle1/distance_traveled', 10)   
        self.x_pos = 0 #starting x 
        self.y_pos = 0 #startiing y 
        self.start = 1 #starting flag

        self.x_total = 0 #total x 
        self.y_total = 0 #total y    

    def distance(self, msg: Pose):
        if (self.start == 1):
            self.x_pos = msg.x
            self.y_pos = msg.y    
            self.start = 0
            return   
        # kunes 18 to 23 basicaly set the first value to the value of the turtle and then turn the flag off so the code can run
        dx = abs(msg.x - self.x_pos)
        dy = abs(msg.y - self.y_pos)

        self.x_total += dx
        self.y_total += dy
        total = self.x_total + self.y_total

        distance_msg = Float32()
        distance_msg.data = total
        self.publisher_.publish(distance_msg)
        

        self.x_pos = msg.x
        self.y_pos = msg.y 

 #line 25 to 38 work by taking the distance the turtle is at and then subtracting it from its old position. this is then stored in dx and dy,
 # this distance than incraments to the total which then gets summed and printed
def main(args=None):
    rclpy.init(args=args)
    node = SquareDistance()               
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
