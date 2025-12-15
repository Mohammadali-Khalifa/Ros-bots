import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist

ID_TO_COLOR = {1: 'blue', 2: 'pink', 3: 'green', 4: 'red'}

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

        # Topics
        self.meas_sub = self.create_subscription(Int32MultiArray, 'marker_measurements', self.meas_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_pub = self.create_publisher(String, 'target_request', 10)   # tell detector pickup/dropoff
        self.grip_pub = self.create_publisher(String, 'gripper_cmd', 10)
        self.mode = 'AUTONOMOUS'
        self.mode_sub = self.create_subscription(String, 'mode_request', self.mode_cb, 10)

        self.last_meas = (-1, -1, 0)  # cx, w, color_id
        self.state = 'SEARCH_PICKUP'
        self.state_ts = time.time()

        self.get_logger().info('Pickup/Delivery FSM started')

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def meas_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 3:
            self.last_meas = (int(msg.data[0]), int(msg.data[1]), int(msg.data[2]))

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
        if not self.param_true('use_gripper'):
            return
        msg = String()
        msg.data = cmd
        self.grip_pub.publish(msg)

    def param_true(self, name: str) -> bool:
        val = self.get_parameter(name).value
        return str(val).strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on')


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

        # linear: move forward if too small, backward if too big (optional)
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

        cx, w, cid = self.last_meas
        seen_color = ID_TO_COLOR.get(cid, '')

        if self.state == 'SEARCH_PICKUP':
            self.target_request('pickup')
            if seen_color == pickup_color and cx >= 0:
                self.set_state('APPROACH_PICKUP')
            else:
                # slow spin search
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = 0.35
                self.cmd_pub.publish(t)

        elif self.state == 'APPROACH_PICKUP':
            self.target_request('pickup')
            if seen_color != pickup_color or cx < 0:
                self.set_state('SEARCH_PICKUP')
                return

            close_enough = self.approach_control(cx, w)
            if close_enough:
                self.stop()
                self.set_state('GRAB')

        elif self.state == 'GRAB':
            self.target_request('pickup')
            use_lift = self.param_true('use_lift')
            pre_open = float(self.get_parameter('pre_open_seconds').value)
            action_s = float(self.get_parameter('action_seconds').value)
            lift_s = float(self.get_parameter('lift_action_seconds').value)

            # Sequence:
            # open -> down -> close -> up (optional)
            if use_lift:
                if self.elapsed() < pre_open:
                    self.gripper('open')
                elif self.elapsed() < pre_open + lift_s:
                    self.gripper('down')
                elif self.elapsed() < pre_open + lift_s + action_s:
                    self.gripper('close')
                elif self.elapsed() < pre_open + lift_s + action_s + lift_s:
                    self.gripper('up')
                else:
                    self.set_state('SEARCH_DROPOFF')
            else:
                if self.elapsed() < action_s:
                    self.gripper('close')
                else:
                    self.set_state('SEARCH_DROPOFF')

        elif self.state == 'SEARCH_DROPOFF':
            self.target_request('dropoff')
            if seen_color == dropoff_color and cx >= 0:
                self.set_state('APPROACH_DROPOFF')
            else:
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = 0.35
                self.cmd_pub.publish(t)

        elif self.state == 'APPROACH_DROPOFF':
            self.target_request('dropoff')
            if seen_color != dropoff_color or cx < 0:
                self.set_state('SEARCH_DROPOFF')
                return

            close_enough = self.approach_control(cx, w)
            if close_enough:
                self.stop()
                self.set_state('RELEASE')

        elif self.state == 'RELEASE':
            self.target_request('dropoff')
            use_lift = self.param_true('use_lift')
            action_s = float(self.get_parameter('action_seconds').value)
            lift_s = float(self.get_parameter('lift_action_seconds').value)

            if use_lift:
                # down -> open -> up
                if self.elapsed() < lift_s:
                    self.gripper('down')
                elif self.elapsed() < lift_s + action_s:
                    self.gripper('open')
                elif self.elapsed() < lift_s + action_s + lift_s:
                    self.gripper('up')
                else:
                    self.set_state('DONE')
            else:
                if self.elapsed() < action_s:
                    self.gripper('open')
                else:
                    self.set_state('DONE')

        elif self.state == 'DONE':
            self.target_request('')
            self.stop()
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
