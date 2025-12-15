import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeInput(Node):
    def __init__(self):
        super().__init__('mode_input_node')
        self.pub = self.create_publisher(String, 'mode_request', 10)

        self.get_logger().info("Mode keys: (a)=AUTONOMOUS, (t)=TELEOP, (s)=STOP")
        t = threading.Thread(target=self._input_thread, daemon=True)
        t.start()

    def _input_thread(self):
        while rclpy.ok():
            try:
                s = input().strip().lower()
            except EOFError:
                return

            msg = String()
            if s == 'a':
                msg.data = 'AUTONOMOUS'
            elif s == 't':
                msg.data = 'TELEOP'
            elif s == 's':
                msg.data = 'STOP'
            else:
                continue
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = ModeInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
