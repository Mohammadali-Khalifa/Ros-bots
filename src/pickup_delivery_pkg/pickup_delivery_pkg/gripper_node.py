import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GripperSerial(Node):
    def __init__(self):
        super().__init__('gripper_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('write_delay_s', 0.02)  # keep serial stable

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(2.0)  # Arduino reset on connect
            self.get_logger().info(f'Connected to Arduino on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {port}: {e}')

        self.sub = self.create_subscription(String, 'gripper_cmd', self.cmd_cb, 10)

    def cmd_cb(self, msg: String):
        if self.ser is None:
            return
        cmd = msg.data.strip().lower()
        if cmd not in ('open', 'close', 'up', 'down', 'stop'):
            return

        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
            time.sleep(float(self.get_parameter('write_delay_s').value))
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


def main():
    rclpy.init()
    node = GripperSerial()
    rclpy.spin(node)
    if node.ser:
        try:
            node.ser.close()
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()
