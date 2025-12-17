import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

#node for extracting image info 
class ImageInfo(Node):
    def __init__(self):
        super().__init__('image_info')
        self.bridge = CvBridge()  # converts ROS images to OpenCV
        self.sub = self.create_subscription(Image, 'image_filtered', self.image_cb, 10) #subscribe to filtered image
        self.pub = self.create_publisher(Int32MultiArray, 'image_info', 10) #publish image info
    def image_cb(self, msg: Image):
        #callback for filtered image to calculate horizontal center and width of ball
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_px = -1 #default values
        width_px = 0
        if contours:
            largest = max(contours, key=cv2.contourArea) #select largest
            x, y, w, h = cv2.boundingRect(largest) #get bounding box
            center_px = x + (w // 2)   # x-center of bounding box
            width_px = w               # width of bounding box
        # publish the info
        out = Int32MultiArray()
        out.data = [int(center_px), int(width_px)]
        self.pub.publish(out) #publish image info
def main(args=None):
    rclpy.init(args=args)
    node = ImageInfo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
