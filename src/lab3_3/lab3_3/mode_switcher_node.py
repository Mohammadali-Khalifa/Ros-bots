import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ModeSwitcherNode(Node):
    def __init__(self):
        super().__init__('mode_switcher_node')

        self.valid_modes = ['TELEOP', 'AUTONOMOUS', 'STOP', 'ESTOP']
        self.current_mode = 'TELEOP'

        self.teleop_cmd = Twist()
        self.auto_cmd = Twist()

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, 'current_mode', 10)

        self.create_subscription(Twist, 'teleop_cmd_vel', self.on_teleop, 10)
        self.create_subscription(Twist, 'auto_cmd_vel', self.on_auto, 10)
        self.create_subscription(String, 'mode_request', self.on_mode_request, 10)

        self.create_timer(0.05, self.tick)
        self.get_logger().info('mode_switcher_node started (TELEOP)')

    def on_teleop(self, msg):
        self.teleop_cmd = msg

    def on_auto(self, msg):
        self.auto_cmd = msg

    def on_mode_request(self, msg):
        desired = msg.data.strip().upper()
        if desired in self.valid_modes and desired != self.current_mode:
            self.current_mode = desired
            self.get_logger().info('Mode changed to ' + self.current_mode)

    def tick(self):
        out = Twist()
        if self.current_mode == 'AUTONOMOUS':
            out = self.auto_cmd
        elif self.current_mode == 'TELEOP':
            out = self.teleop_cmd
        self.pub_mode.publish(String(data=self.current_mode))
        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    node = ModeSwitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
