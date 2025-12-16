import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray, String

COLOR_ID = { 'blue': 1, 'pink': 2, 'green': 3, 'red': 4 }
SHAPE_ID = { 'unknown': 0, 'rect': 1, 'round': 2 }

class ColorFilter(Node):
    def __init__(self):
        super().__init__('color_filter')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.filter, 10) #subscribes to image_filter used by color_filter
        self.pub = self.create_publisher(Image, 'image_filtered', 10) #publishes to image_filter used by color_filter

        self.meas_pub = self.create_publisher(Int32MultiArray, 'marker_measurements', 10)
        self.target_sub = self.create_subscription(String, 'target_request', self.target_cb, 10)
        self.target_phase = ''  # "pickup" / "dropoff" / "object" / ""

        self.pickup_color = 'blue'
        self.dropoff_color = 'pink'

    def target_cb(self, msg: String):
        self.target_phase = msg.data.strip().lower()

    def _mask_for_color(self, hsv, cname):
        if cname == 'blue':
            return cv2.inRange(hsv, (41, 80, 108), (126, 255, 255))
        if cname == 'pink':
            return cv2.inRange(hsv, (140, 60, 60), (179, 255, 255))
        if cname == 'green':
            return cv2.inRange(hsv, (35, 60, 40), (85, 255, 255))
        if cname == 'red':
            m1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
            m2 = cv2.inRange(hsv, (170, 70, 50), (179, 255, 255))
            return m1 | m2
        return np.zeros((hsv.shape[0], hsv.shape[1]), dtype=np.uint8)

    def _classify_shape(self, contour):
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
        if 4 <= v <= 6 and circularity < 0.90:
            return 'rect'
        if circularity >= 0.80 or v >= 7:
            return 'round'
        return 'unknown'

    def filter(self, image_message): # convert ROS image to OpenCV
        bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (95, 120, 60), (130, 255, 255))
        kernel = np.ones((7,7), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        #lines 60 to 66 basicly do the filtering and changing the HSV numbers of low and high can filter more or less (creates a bit mask for the color)

        preferred = None
        if self.target_phase == 'pickup':
            preferred = self.pickup_color
        elif self.target_phase == 'dropoff':
            preferred = self.dropoff_color

        chosen_color = 'blue'
        if preferred in ('blue', 'pink', 'green', 'red'):
            chosen_color = preferred

        mask = self._mask_for_color(hsv, chosen_color)
        kernel = np.ones((7,7), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        clean_mask = np.zeros_like(mask)
        center_px = -1
        width_px = -1
        cid = 0
        sid = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            min_area = 500
            if cv2.contourArea(largest) > min_area:
                cv2.drawContours(clean_mask, [largest], -1, 255, thickness=-1)
                x, y, w, h = cv2.boundingRect(largest)
                center_px = x + (w // 2)
                width_px = w
                cid = int(COLOR_ID.get(chosen_color, 0))
                shape_name = self._classify_shape(largest)
                sid = int(SHAPE_ID.get(shape_name, 0))
            else:
                clean_mask = mask
        else:
            clean_mask = mask
        ros_img = self.bridge.cv2_to_imgmsg(clean_mask, "mono8")
        self.pub.publish(ros_img)
        #lines 104 to 106 are form slides 24-25 of computer vision and then it publshes the images

        meas = Int32MultiArray()
        meas.data = [int(center_px), int(width_px), int(cid), int(sid)]
        self.meas_pub.publish(meas)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
