import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class FSMCode(Node):
    """
    Modes:
      TELEOP: forward teleop/cmd_vel  -> abot/cmd_vel
      AUTO:   forward auto/cmd_vel    -> abot/cmd_vel
      ESTOP:  publish zeros
    """

    def __init__(self):
        super().__init__('abot_fsm')

        self.current_mode = 'TELEOP'
        self.estop_active = False

        self.teleop_cmd = Twist()
        self.auto_cmd = Twist()

        # SUBSCRIPTIONS (all RELATIVE so namespace works)
        self.create_subscription(Twist, 'teleop/cmd_vel', self.teleop_cb, 10)
        self.create_subscription(Twist, 'auto/cmd_vel', self.auto_cb, 10)
        self.create_subscription(String, 'mode', self.mode_cb, 10)
        self.create_subscription(Bool, 'estop', self.estop_cb, 10)

        # PUBLISH to what the ABOT interface listens to
        self.cmd_pub = self.create_publisher(Twist, 'abot/cmd_vel', 10)

        # 20 Hz update
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('FSM started in TELEOP mode')

    def teleop_cb(self, msg: Twist):
        self.teleop_cmd = msg

    def auto_cb(self, msg: Twist):
        self.auto_cmd = msg

    def mode_cb(self, msg: String):
        key = msg.data.strip().lower()
        if key == 't':
            self.current_mode = 'TELEOP'
            self.get_logger().info('Mode -> TELEOP')
        elif key == 'a':
            self.current_mode = 'AUTO'
            self.get_logger().info('Mode -> AUTO')
        elif key == 'e':
            self.current_mode = 'ESTOP'
            self.get_logger().info('Mode -> ESTOP')
        else:
            self.get_logger().warn(f'Unknown mode key: "{msg.data}"')

    def estop_cb(self, msg: Bool):
        self.estop_active = msg.data

    def update(self):
        cmd = Twist()  # default zeros

        if self.estop_active or self.current_mode == 'ESTOP':
            pass  # keep zeros
        elif self.current_mode == 'TELEOP':
            cmd = self.teleop_cmd
        elif self.current_mode == 'AUTO':
            cmd = self.auto_cmd

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FSMCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
