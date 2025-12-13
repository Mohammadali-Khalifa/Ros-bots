import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

class ImageInfo(Node):
    def __init__(self):
        super().__init__('image_info')

        self.bridge = CvBridge()  # converts ROS images to OpenCV

        # subscribes to image_filtered (mono mask: white=ball, black=background)
        self.sub = self.create_subscription(Image, 'image_filtered', self.image_cb, 10)

        # publishes [center_px, width_px]
        self.pub = self.create_publisher(Int32MultiArray, 'image_info', 10)


    def image_cb(self, msg: Image):
        # convert ROS mono8 image -> OpenCV mask
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # find contours on the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        center_px = 0
        width_px = 0

        # if we found something, take the biggest blob (assume ball)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            center_px = x + (w // 2)   # x-center of bounding box
            width_px = w               # width of bounding box

        # publish the info
        out = Int32MultiArray()
        out.data = [int(center_px), int(width_px)]
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ImageInfo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
