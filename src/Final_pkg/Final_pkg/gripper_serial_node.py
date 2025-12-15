import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class GripperSerial(Node):
    def __init__(self):
        super().__init__('gripper_serial')
        
        self.port = '/dev/ttyACM0'
        self.baud = 115200

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

        self.sub = self.create_subscription(
            String,
            'gripper_cmd',
            self.cmd_callback,
            10
        )

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
