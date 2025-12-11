import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class unitconvert(Node):
    def __init__(self):
        super().__init__('unit_convert')

        self.declare_parameter('user_input', 'smoots')
        self.declare_parameter('input', 'turtle1/distance_traveled')
        self.declare_parameter('output', 'converted_distance')

        old_unit = self.get_parameter('input').get_parameter_value().string_value
        new_unit = self.get_parameter('output').get_parameter_value().string_value 

        self.subscription = self.create_subscription(Float32, old_unit, self.callback, 10)
        self.publisher_ = self.create_publisher(Float32, new_unit, 10)   
        #lines 7-17 setup the input, output type and sets up what is to be read by the node. 
        #it than saves the input as the old_unit and than the output as the new_unit
        #it than sets it so both the input and output are of text (float32)

    def callback(self, msg: Float32):
        meters = float(msg.data)
        input_units = self.get_parameter('user_input').get_parameter_value().string_value #reads typing input and saved it to input_unit

        if input_units == 'meter':
            New_output = meters
        elif input_units == 'feet':
            New_output = meters * 3.2808399
        elif input_units == 'smoot':
            New_output = meters * 0.58761312
        else:
            New_output = meters * 0.58761312 # starts as smoots

        Converted_distance = Float32()
        Converted_distance.data = New_output
        self.publisher_.publish(Converted_distance)

        #Lines 25-25 take the user input and sets it to the input_unit
        #lins 27-34 is an if else statment that checks what unit it is and than prints it out in that unit 

def main(args=None):
    rclpy.init(args=args)
    node = unitconvert()               
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

