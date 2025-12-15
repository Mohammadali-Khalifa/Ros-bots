import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist

ID_TO_COLOR = {1: 'blue', 2: 'pink', 3: 'green', 4: 'red'}

# Shape IDs (4th element in marker_measurements)
# 0 = unknown, 1 = rect/square-like, 2 = round-like
RECT_SHAPE_ID = 1
ROUND_SHAPE_ID = 2

class PickupDeliveryFSM(Node):
    def __init__(self):
        super().__init__('pickup_delivery_fsm_node')

        # Params
        self.declare_parameter('pickup_color', 'blue')
        self.declare_parameter('dropoff_color', 'pink')

        self.declare_parameter('image_width', 640)

        # control behavior
        self.declare_parameter('center_tol_px', 35)
        self.declare_parameter('desired_width', 140)      # size when "close enough"
        self.declare_parameter('width_tol', 15)

        self.declare_parameter('k_turn', 0.0035)          # P control for angular
        self.declare_parameter('k_fwd', 0.006)            # P control for linear
        self.declare_parameter('max_lin', 0.20)
        self.declare_parameter('max_ang', 1.0)

        # mode behavior
        self.declare_parameter('force_autonomous', False)

        # gripper behavior
        self.declare_parameter('use_gripper', True)
        self.declare_parameter('use_lift', False)
        self.declare_parameter('pre_open_seconds', 0.4)
        self.declare_parameter('action_seconds', 0.8)     # open/close duration
        self.declare_parameter('lift_action_seconds', 0.8)

        # Manual phases (no driving; user uses gripper teleop only)
        self.declare_parameter('manual_grab_seconds', 8.0)
        self.declare_parameter('manual_release_seconds', 6.0)

        # Topics
        self.meas_sub = self.create_subscription(Int32MultiArray, 'marker_measurements', self.meas_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_pub = self.create_publisher(String, 'target_request', 10)   # tell detector pickup/dropoff/object
        self.grip_pub = self.create_publisher(String, 'gripper_cmd', 10)
        self.mode = 'AUTONOMOUS'
        self.mode_sub = self.create_subscription(String, 'mode_request', self.mode_cb, 10)

        self.last_meas = (-1, -1, 0, 0)  # cx, w, color_id, shape_id
        self.state = 'SEARCH_PICKUP'
        self.state_ts = time.time()

        self.get_logger().info('Pickup/Delivery FSM started')

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def meas_cb(self, msg: Int32MultiArray):
        # Backward compatible: allow either [cx,w,cid] or [cx,w,cid,shape]
        if len(msg.data) >= 4:
            self.last_meas = (int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3]))
        elif len(msg.data) >= 3:
            self.last_meas = (int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), 0)

    def mode_cb(self, msg: String):
        m = msg.data.strip().upper()
        if m in ('AUTONOMOUS', 'TELEOP', 'STOP'):
            if m != self.mode:
                self.get_logger().info(f'Mode: {self.mode} -> {m}')
            self.mode = m
            if m in ('TELEOP', 'STOP'):
                self.stop()

    def set_state(self, s: str):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
        self.state = s
        self.state_ts = time.time()

    def elapsed(self) -> float:
        return time.time() - self.state_ts

    def stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def target_request(self, phase: str):
        msg = String()
        msg.data = phase
        self.target_pub.publish(msg)

    def gripper(self, cmd: str):
        val = self.get_parameter('use_gripper').value
        use_gripper = (str(val).strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on'))
        if not use_gripper:
            return
        msg = String()
        msg.data = cmd
        self.grip_pub.publish(msg)

    def approach_control(self, cx: int, w: int):
        img_w = int(self.get_parameter('image_width').value)
        center = img_w // 2

        center_tol = int(self.get_parameter('center_tol_px').value)
        desired_w = int(self.get_parameter('desired_width').value)
        width_tol = int(self.get_parameter('width_tol').value)

        k_turn = float(self.get_parameter('k_turn').value)
        k_fwd = float(self.get_parameter('k_fwd').value)

        max_lin = float(self.get_parameter('max_lin').value)
        max_ang = float(self.get_parameter('max_ang').value)

        if cx < 0 or w < 0:
            self.stop()
            return False

        err_x = (center - cx)
        err_w = (desired_w - w)

        # angular: turn toward target
        ang = k_turn * err_x

        # linear: move forward if too small, backward if too big
        lin = k_fwd * err_w

        # deadbands
        if abs(err_x) < center_tol:
            ang = 0.0
        if abs(err_w) < width_tol:
            lin = 0.0

        # clamp
        lin = max(-max_lin, min(max_lin, lin))
        ang = max(-max_ang, min(max_ang, ang))

        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

        close_enough = (abs(err_x) < center_tol) and (abs(err_w) < width_tol)
        return close_enough

    def tick(self):
        if self.mode != 'AUTONOMOUS':
            if self.mode == 'STOP':
                self.stop()
            return


        pickup_color = self.get_parameter('pickup_color').value.strip().lower()
        dropoff_color = self.get_parameter('dropoff_color').value.strip().lower()

        cx, w, cid, shape_id = self.last_meas
        seen_color = ID_TO_COLOR.get(cid, '')

        if self.state == 'SEARCH_PICKUP':
            self.target_request('pickup')
            # pickup marker must be correct color AND rect-like
            if seen_color == pickup_color and cx >= 0 and shape_id == RECT_SHAPE_ID:
                self.set_state('APPROACH_PICKUP')
            else:
                # slow spin search
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = 0.35
                self.cmd_pub.publish(t)

        elif self.state == 'APPROACH_PICKUP':
            self.target_request('pickup')
            if seen_color != pickup_color or cx < 0 or shape_id != RECT_SHAPE_ID:
                self.set_state('SEARCH_PICKUP')
                return

            close_enough = self.approach_control(cx, w)
            if close_enough:
                self.stop()
                self.set_state('SEARCH_OBJECT')

        elif self.state == 'SEARCH_OBJECT':
            self.target_request('object')
            if cx >= 0 and shape_id == ROUND_SHAPE_ID:
                self.stop()
                self.set_state('WAIT_FOR_GRAB')
            else:
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = 0.40
                self.cmd_pub.publish(t)

        elif self.state == 'WAIT_FOR_GRAB':
            self.target_request('object')
            self.stop()
            wait_s = float(self.get_parameter('manual_grab_seconds').value)
            if self.elapsed() > wait_s:
                self.set_state('SEARCH_DROPOFF')

        elif self.state == 'SEARCH_DROPOFF':
            self.target_request('dropoff')
            if seen_color == dropoff_color and cx >= 0 and shape_id == RECT_SHAPE_ID:
                self.set_state('APPROACH_DROPOFF')
            else:
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = 0.35
                self.cmd_pub.publish(t)

        elif self.state == 'APPROACH_DROPOFF':
            self.target_request('dropoff')
            if seen_color != dropoff_color or cx < 0 or shape_id != RECT_SHAPE_ID:
                self.set_state('SEARCH_DROPOFF')
                return

            close_enough = self.approach_control(cx, w)
            if close_enough:
                self.stop()
                self.set_state('WAIT_FOR_RELEASE')

        elif self.state == 'WAIT_FOR_RELEASE':
            self.target_request('dropoff')
            self.stop()
            wait_s = float(self.get_parameter('manual_release_seconds').value)
            if self.elapsed() > wait_s:
                self.set_state('SEARCH_PICKUP')

        else:
            self.stop()
            self.set_state('SEARCH_PICKUP')


def main():
    rclpy.init()
    node = PickupDeliveryFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
