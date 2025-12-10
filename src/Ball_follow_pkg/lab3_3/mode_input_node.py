import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeInputNode(Node):
    def __init__(self):
        super().__init__('mode_input_node')
        self.pub_req = self.create_publisher(String, 'mode_request', 10)

    def run(self):
        mapping = {
            't': 'TELEOP',
            'a': 'AUTONOMOUS',
            's': 'STOP',
            'e': 'ESTOP',
        }
        try:
            while True:
                try:
                    ch = input('t/a/s/e, q to quit > ').strip().lower()
                except EOFError:
                    break
                if not ch:
                    continue
                if ch == 'q':
                    break
                mode = mapping.get(ch)
                if mode is None:
                    print('Use t/a/s/e or q')
                    continue
                self.pub_req.publish(String(data=mode))
                print('Requested', mode)
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = ModeInputNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
