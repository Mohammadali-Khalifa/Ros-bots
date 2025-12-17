import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#import control libraries
try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

#message displayed for user
HELP = """
Gripper keyboard control:
  o = open
  c = close
  u = up
  d = down
  s = stop
  q = quit
"""

#node for controlling gripper using key presses
class GripperKeyboardNode(Node):
    def __init__(self):
        super().__init__('gripper_keyboard_node')
        self.pub = self.create_publisher(String, 'gripper_cmd', 10) #publish gripper command string
        self.get_logger().info(HELP.strip()) #print instructions

    def _get_key(self):
        #read kepress from terminal
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
        #mapping of gripper commands and publishing
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
