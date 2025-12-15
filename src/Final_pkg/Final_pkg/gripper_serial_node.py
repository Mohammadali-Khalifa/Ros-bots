import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


class GripperSerialNode(Node):
    def __init__(self):
        super().__init__('gripper_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)

        self.ser = None
        self._connect()

        self.create_subscription(String, 'gripper_cmd', self.on_cmd, 10)
        self.get_logger().info('gripper_serial_node started')

    def _connect(self):
        if serial is None:
            self.get_logger().error("pyserial missing. Install: sudo apt install python3-serial")
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.get_logger().info(f'Connected: {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open {self.port}: {e}')
            self.ser = None

    def on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in ('open', 'close', 'up', 'down', 'stop'):
            self.get_logger().warn(f'Unknown gripper_cmd: "{cmd}"')
            return

        if self.ser is None:
            self._connect()
            if self.ser is None:
                return

        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = GripperSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser is not None:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
