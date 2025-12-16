from math import isnan
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from geometry_msgs.msg import Twist

ID_TO_COLOR = {1: 'blue', 2: 'pink', 3: 'green', 4: 'red'}
ID_TO_SHAPE = {0: 'unknown', 1: 'rect', 2: 'round'}

class BallFollow(Node):
    def __init__(self):
        super().__init__('ball_follow_simple')
        self.image_width = 640.0                  # camera image in pixels
        self.target_width = 200.0                   #width of the ball
        self.angular_gain = 1.6                # how much to turn left or right to see the ball
        self.linear_gain = 0.004                # how much to go forward to the ball
        self.center_deadband = 0.15            # for when the ball is centered
        self.width_deadband_px = 30.0           #this is for moving forward and back
        self.min_turn_speed = 0.50            # maintains steady speeds for turning
        self.min_forward_speed = 0.30            #mianitans steady speeds for going forwards and back (reduces the jerking)
        self.turn_first_threshold = 0.25         #how much the bot should be cenetred beofore it goes stright

        self.object_target_width = 260.0          #width of the ball when it is ~15cm away (calibrate if needed)
        self.object_width_deadband_px = 20.0
        self.object_min_forward_speed = 0.08
        self.object_linear_gain = 0.002
        self.object_min_turn_speed = 0.15
        self.object_center_deadband = 0.10
        self.object_turn_first_threshold = 0.20
        
        self.create_subscription(Int32MultiArray, 'image_info', self.image_callback, 10)  #subscibes to image_info to get the ball info
        self.cmd_pub = self.create_publisher(Twist, 'auto/cmd_vel', 10) #publishes to auto/cmd_vel to move the motors

        self.target_pub = self.create_publisher(String, 'target_request', 10)
        self.grip_sub = self.create_subscription(String, 'gripper_cmd', self.gripper_cb, 10)
        self.grip_pub = self.create_publisher(String, 'gripper_cmd', 10)

        self.pickup_color = 'blue'
        self.dropoff_color = 'pink'

        self.state = 'SEARCH_PICKUP'
        self.last_gripper_cmd = ''

        self._grip_seq = ''
        self._grip_step = 0
        self._grip_step_t = self.get_clock().now()
        self._grip_init_done = False
        self.grip_timer = self.create_timer(0.05, self._gripper_update)

    def gripper_cb(self, msg: String):
        self.last_gripper_cmd = msg.data.strip().lower()

    def _gripper_send(self, cmd: str):
        self.grip_pub.publish(String(data=cmd))
        self.last_gripper_cmd = cmd.strip().lower()

    def _gripper_start(self, seq: str):
        if self._grip_seq != seq:
            self._grip_seq = seq
            self._grip_step = 0
            self._grip_step_t = self.get_clock().now()

    def _gripper_update(self):
        now = self.get_clock().now()

        if not self._grip_init_done:
            if (now - self._grip_step_t).nanoseconds < int(0.5 * 1e9):
                return
            if self._grip_step == 0:
                self._gripper_send('open')
                self._grip_step_t = now
                self._grip_step = 1
                return
            if self._grip_step == 1:
                self._gripper_send('up')
                self._grip_step_t = now
                self._grip_step = 2
                return
            self._grip_init_done = True
            self._grip_seq = ''
            self._grip_step = 0
            return

        if self._grip_seq == 'GRAB':
            if (now - self._grip_step_t).nanoseconds < int(0.7 * 1e9):
                return
            if self._grip_step == 0:
                self._gripper_send('down')
                self._grip_step_t = now
                self._grip_step = 1
                return
            if self._grip_step == 1:
                self._gripper_send('close')
                self._grip_step_t = now
                self._grip_step = 2
                return
            if self._grip_step == 2:
                self._gripper_send('up')
                self._grip_step_t = now
                self._grip_step = 3
                return
            self._grip_seq = ''
            self._grip_step = 0
            return

        if self._grip_seq == 'RELEASE':
            if (now - self._grip_step_t).nanoseconds < int(0.7 * 1e9):
                return
            if self._grip_step == 0:
                self._gripper_send('down')
                self._grip_step_t = now
                self._grip_step = 1
                return
            if self._grip_step == 1:
                self._gripper_send('open')
                self._grip_step_t = now
                self._grip_step = 2
                return
            self._grip_seq = ''
            self._grip_step = 0
            return

    def _publish_target(self, phase: str):
        self.target_pub.publish(String(data=phase))

    def _stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _spin(self):
        cmd = Twist()
        cmd.angular.z = self.min_turn_speed
        self.cmd_pub.publish(cmd)

    def _spin_slow(self):
        cmd = Twist()
        cmd.angular.z = self.object_min_turn_speed
        self.cmd_pub.publish(cmd)

    def image_callback(self, msg):
        cmd = Twist()
        if len(msg.data) < 2:
            self._stop()
            return

        center_px = float(msg.data[0])
        width_px = float(msg.data[1])

        image_center = self.image_width / 2.0
        center_error = 0.0
        if center_px > 0.0 and not isnan(center_px):
            center_error = (center_px - image_center) / image_center

        if width_px <= 0.0 or center_px <= 0.0 or isnan(center_px):
            if self.state == 'SEARCH_PICKUP':
                self._publish_target('pickup')
                self._spin()
                return
            if self.state == 'SEARCH_OBJECT':
                self._publish_target('object')
                self._spin_slow()
                return
            if self.state == 'SEARCH_DROPOFF':
                self._publish_target('dropoff')
                self._spin()
                return

            self._stop()
            return

        if len(msg.data) >= 4:
            color_id = int(msg.data[2])
            shape_id = int(msg.data[3])
            seen_color = ID_TO_COLOR.get(color_id, '')
            seen_shape = ID_TO_SHAPE.get(shape_id, 'unknown')

            close_enough = (abs(center_error) <= self.center_deadband) and (
                abs(self.target_width - width_px) <= self.width_deadband_px
            )

            if self.state == 'SEARCH_PICKUP':
                self._publish_target('pickup')
                if seen_color == self.pickup_color and seen_shape == 'rect':
                    self.state = 'APPROACH_PICKUP'
                else:
                    self._spin()
                return

            if self.state == 'APPROACH_PICKUP':
                self._publish_target('pickup')
                if seen_color != self.pickup_color:
                    self.state = 'SEARCH_PICKUP'
                    self._spin()
                    return

                if abs(center_error) > self.center_deadband:
                    angular_speed = -self.angular_gain * center_error
                    if angular_speed > 0.0:
                        angular_speed = max(angular_speed, self.min_turn_speed)
                    else:
                        angular_speed = min(angular_speed, -self.min_turn_speed)
                    cmd.angular.z = angular_speed

                width_error = self.target_width - width_px
                if abs(width_error) > self.width_deadband_px:
                    linear_speed = self.linear_gain * width_error
                    if abs(center_error) > self.turn_first_threshold:
                        linear_speed = 0.0
                    if 0.0 < linear_speed < self.min_forward_speed:
                        linear_speed = self.min_forward_speed
                    cmd.linear.x = linear_speed

                self.cmd_pub.publish(cmd)
                if close_enough:
                    self._stop()
                    self.state = 'SEARCH_OBJECT'
                return

            if self.state == 'SEARCH_OBJECT':
                self._publish_target('object')
                if seen_shape == 'round':
                    self.state = 'APPROACH_OBJECT'
                else:
                    self._spin_slow()
                return

            if self.state == 'APPROACH_OBJECT':
                self._publish_target('object')
                if seen_shape != 'round':
                    self.state = 'SEARCH_OBJECT'
                    self._spin_slow()
                    return

                obj_center_error = center_error

                if abs(obj_center_error) > self.object_center_deadband:
                    angular_speed = -self.angular_gain * obj_center_error
                    if angular_speed > 0.0:
                        angular_speed = max(angular_speed, self.object_min_turn_speed)
                    else:
                        angular_speed = min(angular_speed, -self.object_min_turn_speed)
                    cmd.angular.z = angular_speed

                width_error = self.object_target_width - width_px
                if abs(width_error) > self.object_width_deadband_px:
                    linear_speed = self.object_linear_gain * width_error
                    if abs(obj_center_error) > self.object_turn_first_threshold:
                        linear_speed = 0.0
                    if 0.0 < linear_speed < self.object_min_forward_speed:
                        linear_speed = self.object_min_forward_speed
                    cmd.linear.x = linear_speed

                self.cmd_pub.publish(cmd)
                close_enough_obj = (abs(obj_center_error) <= self.object_center_deadband) and (
                    abs(self.object_target_width - width_px) <= self.object_width_deadband_px
                )
                if close_enough_obj:
                    self._stop()
                    self.state = 'WAIT_GRAB'
                return

            if self.state == 'WAIT_GRAB':
                self._publish_target('object')
                self._stop()
                self._gripper_start('GRAB')
                if self._grip_seq == '':
                    self.state = 'SEARCH_DROPOFF'
                return

            if self.state == 'SEARCH_DROPOFF':
                self._publish_target('dropoff')
                if seen_color == self.dropoff_color and seen_shape == 'rect':
                    self.state = 'APPROACH_DROPOFF'
                else:
                    self._spin()
                return

            if self.state == 'APPROACH_DROPOFF':
                self._publish_target('dropoff')
                if seen_color != self.dropoff_color:
                    self.state = 'SEARCH_DROPOFF'
                    self._spin()
                    return

                if abs(center_error) > self.center_deadband:
                    angular_speed = -self.angular_gain * center_error
                    if angular_speed > 0.0:
                        angular_speed = max(angular_speed, self.min_turn_speed)
                    else:
                        angular_speed = min(angular_speed, -self.min_turn_speed)
                    cmd.angular.z = angular_speed

                width_error = self.target_width - width_px
                if abs(width_error) > self.width_deadband_px:
                    linear_speed = self.linear_gain * width_error
                    if abs(center_error) > self.turn_first_threshold:
                        linear_speed = 0.0
                    if 0.0 < linear_speed < self.min_forward_speed:
                        linear_speed = self.min_forward_speed
                    cmd.linear.x = linear_speed

                self.cmd_pub.publish(cmd)
                if close_enough:
                    self._stop()
                    self.state = 'WAIT_RELEASE'
                return

            if self.state == 'WAIT_RELEASE':
                self._publish_target('dropoff')
                self._stop()
                self._gripper_start('RELEASE')
                if self._grip_seq == '':
                    self._grip_init_done = False
                    self._grip_step = 0
                    self._grip_step_t = self.get_clock().now()
                    self.state = 'SEARCH_PICKUP'
                return

        if abs(center_error) > self.center_deadband:
            angular_speed = -self.angular_gain * center_error
            if angular_speed > 0.0:
                angular_speed = max(angular_speed, self.min_turn_speed)
            else:
                angular_speed = min(angular_speed, -self.min_turn_speed)
            cmd.angular.z = angular_speed

        width_error = self.target_width - width_px
        if abs(width_error) > self.width_deadband_px:
            linear_speed = self.linear_gain * width_error
            if abs(center_error) > self.turn_first_threshold:
                linear_speed = 0.0
            if 0.0 < linear_speed < self.min_forward_speed:
                linear_speed = self.min_forward_speed
            cmd.linear.x = linear_speed

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
