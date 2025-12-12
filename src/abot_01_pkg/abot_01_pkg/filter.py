import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
class ColorFilter(Node):
    def __init__(self):
        super().__init__('color_filter')
        self.bridge = CvBridge()

        # Subscribe to the raw camera feed (matches your topic list)
        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.cb,
            10
        )

        # Publish filtered image for debugging
        self.pub = self.create_publisher(Image, 'image_filtered', 10)

        self.get_logger().info("ColorFilter node started (target = BLUE)")

    def cb(self, msg: Image):
        # ROS Image -> OpenCV
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV and threshold blue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # These ranges are a decent starting point for a blue ping pong ball
        lower = np.array([90, 80, 50])
        upper = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        # Optional cleanup
        mask = cv2.medianBlur(mask, 5)

        # Convert mask to BGR so it displays nicely
        filtered_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # OpenCV -> ROS Image
        out_msg = self.bridge.cv2_to_imgmsg(filtered_bgr, encoding='bgr8')
        out_msg.header = msg.header

        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = ColorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
