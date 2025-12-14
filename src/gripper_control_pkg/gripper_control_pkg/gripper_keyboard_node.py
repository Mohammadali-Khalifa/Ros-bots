import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None


HELP = """
Gripper keyboard control:
  o = open
  c = close
  u = up
  d = down
  s = stop
  q = quit
"""


class GripperKeyboardNode(Node):
    def __init__(self):
        super().__init__('gripper_keyboard_node')
        self.pub = self.create_publisher(String, 'gripper_cmd', 10)
        self.get_logger().info(HELP.strip())

    def _get_key(self):
        if termios is None or tty is None:
            return None
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def run(self):
        mapping = {'o': 'open', 'c': 'close', 'u': 'up', 'd': 'down', 's': 'stop'}
        while rclpy.ok():
            k = self._get_key()
            if not k:
                continue
            k = k.lower()
            if k == 'q':
                break
            if k in mapping:
                self.pub.publish(String(data=mapping[k]))
                self.get_logger().info(f'gripper_cmd -> {mapping[k]}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperKeyboardNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
