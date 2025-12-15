import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


COLOR_ID = {
    'blue': 1,
    'pink': 2,
    'green': 3,
    'red': 4,
}

# Shape IDs (4th element in marker_measurements)
# 0 = unknown, 1 = rect/square-like, 2 = round-like
SHAPE_ID = {
    'unknown': 0,
    'rect': 1,
    'round': 2,
}


class HSVDetector(Node):
    def __init__(self):
        super().__init__('hsv_detector_node')

        # Params
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('min_area', 250)

        # HSV thresholds (tune on robot)
        self.declare_parameter('blue_low',  [95, 80, 40])
        self.declare_parameter('blue_high', [130, 255, 255])

        self.declare_parameter('pink_low',  [140, 60, 60])
        self.declare_parameter('pink_high', [179, 255, 255])

        self.declare_parameter('green_low', [35, 60, 40])
        self.declare_parameter('green_high',[85, 255, 255])

        self.declare_parameter('red1_low',  [0, 70, 50])
        self.declare_parameter('red1_high', [10, 255, 255])
        self.declare_parameter('red2_low',  [170, 70, 50])
        self.declare_parameter('red2_high', [179, 255, 255])

        # Preferred target from FSM: "pickup" / "dropoff" / "object" / "" (none)
        self.declare_parameter('pickup_color', 'blue')
        self.declare_parameter('dropoff_color', 'pink')
        self.target_phase = ''  # updated via topic

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.target_sub = self.create_subscription(String, 'target_request', self.target_cb, 10)

        self.meas_pub = self.create_publisher(Int32MultiArray, 'marker_measurements', 10)
        self.debug_pub = self.create_publisher(Image, 'image_filtered', 10)

        self.get_logger().info(f'HSV detector listening on: {image_topic}')

    def target_cb(self, msg: String):
        self.target_phase = msg.data.strip().lower()

    def _get_hsv_ranges(self):
        def p(name):
            return self.get_parameter(name).get_parameter_value().integer_array_value

        ranges = {}
        ranges['blue']  = (np.array(p('blue_low')),  np.array(p('blue_high')))
        ranges['pink']  = (np.array(p('pink_low')),  np.array(p('pink_high')))
        ranges['green'] = (np.array(p('green_low')), np.array(p('green_high')))

        # red has two ranges
        ranges['red'] = (
            (np.array(p('red1_low')), np.array(p('red1_high'))),
            (np.array(p('red2_low')), np.array(p('red2_high'))),
        )
        return ranges

    def _mask_for_color(self, hsv, color_name, ranges):
        if color_name == 'red':
            (l1, h1), (l2, h2) = ranges['red']
            return cv2.inRange(hsv, l1, h1) | cv2.inRange(hsv, l2, h2)
        low, high = ranges[color_name]
        return cv2.inRange(hsv, low, high)

    def _classify_shape(self, contour):
        # Simple contour-based shape classification:
        # - rect/square tends to approximate to 4 vertices
        # - circle/ball tends to have many vertices and higher circularity
        area = cv2.contourArea(contour)
        if area <= 0:
            return 'unknown'

        peri = cv2.arcLength(contour, True)
        if peri <= 0:
            return 'unknown'

        circularity = 4.0 * np.pi * area / (peri * peri)

        eps = 0.03 * peri
        approx = cv2.approxPolyDP(contour, eps, True)
        v = len(approx)

        # Rect-ish: 4 vertices (or sometimes 5-6 when noisy) and not too circular
        if 4 <= v <= 6 and circularity < 0.90:
            return 'rect'

        # Round-ish: high circularity OR lots of vertices
        if circularity >= 0.80 or v >= 7:
            return 'round'

        return 'unknown'

    def _find_best_blob(self, mask, min_area):
        # clean mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        best = None
        best_area = 0
        best_bbox = None

        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if area > best_area:
                best_area = area
                best = c
                best_bbox = (x, y, w, h)

        if best is None:
            return None, mask

        x, y, w, h = best_bbox
        cx = x + w // 2
        shape_name = self._classify_shape(best)
        shape_id = int(SHAPE_ID.get(shape_name, 0))
        return (cx, w, shape_id), mask

    def image_cb(self, msg: Image):
        min_area = int(self.get_parameter('min_area').value)
        pickup_color = self.get_parameter('pickup_color').value.strip().lower()
        dropoff_color = self.get_parameter('dropoff_color').value.strip().lower()

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges = self._get_hsv_ranges()

        colors = ['blue', 'pink', 'green', 'red']

        # If FSM requested a phase, prefer that color first
        preferred = None
        if self.target_phase == 'pickup':
            preferred = pickup_color
        elif self.target_phase == 'dropoff':
            preferred = dropoff_color
        else:
            preferred = None

        ordered = colors
        if preferred in colors:
            ordered = [preferred] + [c for c in colors if c != preferred]

        best_color = None
        best_measure = None
        best_area_proxy = -1
        best_mask = None

        # choose best among colors (but prefer requested by ordering)
        for cname in ordered:
            mask = self._mask_for_color(hsv, cname, ranges)
            m, cleaned = self._find_best_blob(mask, min_area)
            if m is None:
                continue

            cx, w, shape_id = m
            # w is a decent proxy for "closeness/size"
            score = w
            if best_measure is None:
                best_measure = (cx, w, shape_id)
                best_color = cname
                best_area_proxy = score
                best_mask = cleaned
            else:
                # If a preferred color is first, keep it unless it is truly tiny vs others
                if cname == preferred:
                    best_measure = (cx, w, shape_id)
                    best_color = cname
                    best_area_proxy = score
                    best_mask = cleaned
                else:
                    if score > best_area_proxy * 1.25:  # only override if clearly better
                        best_measure = (cx, w, shape_id)
                        best_color = cname
                        best_area_proxy = score
                        best_mask = cleaned

        # publish debug image: show chosen mask
        if best_mask is None:
            best_mask = np.zeros((bgr.shape[0], bgr.shape[1]), dtype=np.uint8)

        debug = cv2.cvtColor(best_mask, cv2.COLOR_GRAY2BGR)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

        # publish measurements
        meas = Int32MultiArray()
        if best_measure is None or best_color is None:
            meas.data = [-1, -1, 0, 0]
        else:
            cx, w, shape_id = best_measure
            meas.data = [int(cx), int(w), int(COLOR_ID.get(best_color, 0)), int(shape_id)]
        self.meas_pub.publish(meas)


def main():
    rclpy.init()
    node = HSVDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
