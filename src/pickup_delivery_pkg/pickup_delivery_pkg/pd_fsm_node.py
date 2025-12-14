from math import isnan
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist


class PickupDeliveryFSM(Node):
    def __init__(self):
        super().__init__('pickup_delivery_fsm_node')

        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('pickup_desired_width', 120.0)
        self.declare_parameter('dropoff_desired_width', 120.0)

        self.declare_parameter('search_angular_speed', 0.4)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 1.0)

        self.declare_parameter('kp_ang', 0.8)
        self.declare_parameter('kp_lin', 0.003)

        self.image_width = float(self.get_parameter('image_width').value)

        self.pub_auto = self.create_publisher(Twist, 'auto_cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, 'mode_request', 10)
        self.pub_grip = self.create_publisher(String, 'gripper_cmd', 10)

        self.create_subscription(Float32MultiArray, 'marker_measurements', self.on_meas, 10)

        self.state = 'SEARCH_PICKUP'
        self.last_center = float('nan')
        self.last_width = 0.0
        self.last_color_id = 0.0

        self.grab_sent = False
        self.release_sent = False

        self.create_timer(0.05, self.tick)
        self.get_logger().info('pickup_delivery_fsm_node started')

    def on_meas(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        self.last_center = float(msg.data[0])
        self.last_width = float(msg.data[1])
        self.last_color_id = float(msg.data[2])

    def _clip(self, x, lo, hi):
        return max(lo, min(hi, x))

    def tick(self):
        self.pub_mode.publish(String(data='AUTONOMOUS'))

        center = self.last_center
        width = self.last_width
        cid = self.last_color_id  # 1=pickup, 2=dropoff

        if isnan(center) or width <= 0.0 or cid == 0.0:
            if self.state.startswith('SEARCH'):
                tw = Twist()
                tw.angular.z = float(self.get_parameter('search_angular_speed').value)
                self.pub_auto.publish(tw)
            else:
                self.pub_auto.publish(Twist())
            return

        half = self.image_width / 2.0
        error_norm = (center - half) / half

        kp_ang = float(self.get_parameter('kp_ang').value)
        kp_lin = float(self.get_parameter('kp_lin').value)

        max_lin = float(self.get_parameter('max_linear').value)
        max_ang = float(self.get_parameter('max_angular').value)

        if self.state == 'SEARCH_PICKUP':
            if cid == 1.0:
                self.state = 'APPROACH_PICKUP'
                self.grab_sent = False
            else:
                tw = Twist()
                tw.angular.z = float(self.get_parameter('search_angular_speed').value)
                self.pub_auto.publish(tw)
            return

        if self.state == 'APPROACH_PICKUP':
            if cid != 1.0:
                self.state = 'SEARCH_PICKUP'
                self.pub_auto.publish(Twist())
                return

            desired = float(self.get_parameter('pickup_desired_width').value)
            lin_error = desired - width

            tw = Twist()
            tw.angular.z = self._clip(-kp_ang * error_norm, -max_ang, max_ang)
            tw.linear.x = self._clip(kp_lin * lin_error, 0.0, max_lin)
            self.pub_auto.publish(tw)

            if abs(lin_error) < 10.0 and abs(error_norm) < 0.1:
                self.state = 'GRAB'
                self.pub_auto.publish(Twist())
            return

        if self.state == 'GRAB':
            if not self.grab_sent:
                self.pub_grip.publish(String(data='close'))
                self.grab_sent = True
            self.state = 'SEARCH_DROPOFF'
            return

        if self.state == 'SEARCH_DROPOFF':
            if cid == 2.0:
                self.state = 'APPROACH_DROPOFF'
                self.release_sent = False
            else:
                tw = Twist()
                tw.angular.z = float(self.get_parameter('search_angular_speed').value)
                self.pub_auto.publish(tw)
            return

        if self.state == 'APPROACH_DROPOFF':
            if cid != 2.0:
                self.state = 'SEARCH_DROPOFF'
                self.pub_auto.publish(Twist())
                return

            desired = float(self.get_parameter('dropoff_desired_width').value)
            lin_error = desired - width

            tw = Twist()
            tw.angular.z = self._clip(-kp_ang * error_norm, -max_ang, max_ang)
            tw.linear.x = self._clip(kp_lin * lin_error, 0.0, max_lin)
            self.pub_auto.publish(tw)

            if abs(lin_error) < 10.0 and abs(error_norm) < 0.1:
                self.state = 'RELEASE'
                self.pub_auto.publish(Twist())
            return

        if self.state == 'RELEASE':
            if not self.release_sent:
                self.pub_grip.publish(String(data='open'))
                self.release_sent = True
            self.state = 'DONE'
            return

        if self.state == 'DONE':
            self.pub_mode.publish(String(data='STOP'))
            self.pub_auto.publish(Twist())
            return


def main(args=None):
    rclpy.init(args=args)
    node = PickupDeliveryFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
