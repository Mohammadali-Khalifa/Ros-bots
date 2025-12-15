import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = """
Gripper teleop:
  o : open
  c : close
  u : up
  d : down
  x : stop
  q : quit
"""


class GripperTeleop(Node):
    def __init__(self):
        super().__init__('gripper_teleop_node')
        self.pub = self.create_publisher(String, 'gripper_cmd', 10)
        self.get_logger().info(HELP.strip())

    def publish_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        self.get_logger().info(f'gripper_cmd -> {cmd}')


def getch():
    """Read a single character from stdin (no Enter required)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    rclpy.init()
    node = GripperTeleop()

    try:
        while rclpy.ok():
            ch = getch().lower()

            if ch == 'o':
                node.publish_cmd('open')
            elif ch == 'c':
                node.publish_cmd('close')
            elif ch == 'u':
                node.publish_cmd('up')
            elif ch == 'd':
                node.publish_cmd('down')
            elif ch == 'x':
                node.publish_cmd('stop')
            elif ch == 'q':
                break
            elif ch in ('h', '?'):
                node.get_logger().info(HELP.strip())

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
