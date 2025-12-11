import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
class ImageInfo(Node):
   def __init__(self):
       super().__init__('image_info')
       self.bridge = CvBridge()
       self.create_subscription(
           Image,
           'image_filtered',
           self.image_cb,
           10
       )
       self.pub_info = self.create_publisher(
           Int32MultiArray,
           'image_info',
           10
       )
       self.get_logger().info('ImageInfo node started.')
   def image_cb(self, msg: Image):
       mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
       # Find contours in the mask
       contours, _ = cv2.findContours(
           mask,
           cv2.RETR_EXTERNAL,
           cv2.CHAIN_APPROX_SIMPLE
       )
       center_px = 0
       width_px = 0
       if contours:
           # Take largest contour (assume it's the ball)
           largest = max(contours, key=cv2.contourArea)
           x, y, w, h = cv2.boundingRect(largest)
           center_px = x + w // 2
           width_px = w
       info_msg = Int32MultiArray()
       info_msg.data = [center_px, width_px]
       self.pub_info.publish(info_msg)
def main(args=None):
   rclpy.init(args=args)
   node = ImageInfo()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
if __name__ == '__main__':
   main()
