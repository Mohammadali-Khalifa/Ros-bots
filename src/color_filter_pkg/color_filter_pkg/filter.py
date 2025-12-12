import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFilter(Node):
    def __init__(self):
        super().__init__('color_filter')

        self.pub = self.create_publisher(Image, 'image_filtered', 10)  # publishes filtered image
        self.sub = self.create_subscription(Image, 'image_raw', self.filter, 10)  # subscribes to raw image

        self.bridge = CvBridge()

        self.declare_parameter('target_color', 'blue')  # sets default to blue
        self.declare_parameter('erode_dilate_iters', 2) # number of erode/dilate iterations

        self.declare_parameter('kernel_size', 7)

    def filter(self, image_message):
        # convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        color = self.get_parameter('target_color').value

        if color == 'blue':
            mask = cv2.inRange(hsv, (90, 80, 40), (140, 255, 255))
        elif color == 'pink':
            mask = cv2.inRange(hsv, (140, 40, 40), (180, 255, 255))
        elif color == 'white':
            mask = cv2.inRange(hsv, (0, 0, 180), (40, 150, 255))
        else:
            mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 255))

        # Use param for morphology iterations
        iters = int(self.get_parameter('erode_dilate_iters').value)

        # Kernel size
        k = int(self.get_parameter('kernel_size').value)
        if k < 1:
            k = 1
        if k % 2 == 0:
            k += 1

        kernel = np.ones((k, k), np.uint8)
        if iters > 0:
            mask = cv2.erode(mask, kernel, iterations=iters)
            mask = cv2.dilate(mask, kernel, iterations=iters)

        # Apply mask
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        ros_img = self.bridge.cv2_to_imgmsg(result, "bgr8")
        ros_img.header = image_message.header  # preserve timestamp/frame
        self.pub.publish(ros_img)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
