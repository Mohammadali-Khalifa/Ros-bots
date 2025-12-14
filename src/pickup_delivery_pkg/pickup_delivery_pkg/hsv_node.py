import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


def hsv_range_for(name: str):
    name = name.lower().strip()
    if name == 'blue':
        return (90, 80, 40), (140, 255, 255)
    if name == 'pink':
        return (140, 40, 40), (180, 255, 255)
    if name == 'green':
        return (35, 80, 40), (85, 255, 255)
    if name == 'yellow':
        return (20, 100, 60), (35, 255, 255)
    if name == 'orange':
        return (5, 120, 60), (20, 255, 255)
    return (0, 0, 0), (180, 255, 255)


class MultiColorDetectorNode(Node):
    def __init__(self):
        super().__init__('multi_color_detector_node')

        self.declare_parameter('pickup_color', 'blue')
        self.declare_parameter('dropoff_color', 'pink')
        self.declare_parameter('morph_iterations', 2)

        self.bridge = CvBridge()
        self.pub_meas = self.create_publisher(Float32MultiArray, 'marker_measurements', 10)
        self.pub_dbg = self.create_publisher(Image, 'image_filtered', 10)

        self.create_subscription(Image, 'image_raw', self.on_image, 10)
        self.get_logger().info('multi_color_detector_node started')

    def _measure_mask(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        shape = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(shape)
        if area < 200:
            return None

        x, y, w, h = cv2.boundingRect(shape)
        center = x + (w / 2.0)
        width = float(w)
        return center, width

    def on_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        pickup = str(self.get_parameter('pickup_color').value)
        dropoff = str(self.get_parameter('dropoff_color').value)
        iters = int(self.get_parameter('morph_iterations').value)

        p_lo, p_hi = hsv_range_for(pickup)
        d_lo, d_hi = hsv_range_for(dropoff)

        mask_p = cv2.inRange(hsv, p_lo, p_hi)
        mask_d = cv2.inRange(hsv, d_lo, d_hi)

        kernel = np.ones((7, 7), np.uint8)
        if iters > 0:
            mask_p = cv2.erode(mask_p, kernel, iterations=iters)
            mask_p = cv2.dilate(mask_p, kernel, iterations=iters)
            mask_d = cv2.erode(mask_d, kernel, iterations=iters)
            mask_d = cv2.dilate(mask_d, kernel, iterations=iters)

        meas_p = self._measure_mask(mask_p)
        meas_d = self._measure_mask(mask_d)

        color_id = 0.0
        center = float('nan')
        width = 0.0

        if meas_p and meas_d:
            if meas_p[1] >= meas_d[1]:
                color_id = 1.0
                center, width = meas_p
                dbg = cv2.bitwise_and(cv_image, cv_image, mask=mask_p)
            else:
                color_id = 2.0
                center, width = meas_d
                dbg = cv2.bitwise_and(cv_image, cv_image, mask=mask_d)
        elif meas_p:
            color_id = 1.0
            center, width = meas_p
            dbg = cv2.bitwise_and(cv_image, cv_image, mask=mask_p)
        elif meas_d:
            color_id = 2.0
            center, width = meas_d
            dbg = cv2.bitwise_and(cv_image, cv_image, mask=mask_d)
        else:
            dbg = cv2.bitwise_and(cv_image, cv_image, mask=(mask_p | mask_d))

        out = Float32MultiArray()
        out.data = [float(center), float(width), float(color_id)]
        self.pub_meas.publish(out)

        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
        dbg_msg.header = msg.header
        self.pub_dbg.publish(dbg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
