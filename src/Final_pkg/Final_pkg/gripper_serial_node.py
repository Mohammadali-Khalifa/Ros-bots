import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

#node for controlling gripper using ESP32
class GripperSerial(Node):
    def __init__(self):
        super().__init__('gripper_serial')
        
        self.port = '/dev/ttyACM1' #port
        self.baud = 115200 #baud rate

        #attempt to open serial connection
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except Exception as e: #handle connection failure
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

        self.sub = self.create_subscription( #subscribe to gripper command topic
            String,
            'gripper_cmd',
            self.cmd_callback,
            10
        )

    #callback for gripper commands entered to send to serial port
    def cmd_callback(self, msg):
        if self.ser:
            cmd = msg.data.strip()
            self.ser.write((cmd + '\n').encode())
            self.get_logger().info(f'Sent: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
