import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
class ColorFilter(Node):
   def __init__(self):
       super().__init__('color_filter')
       self.bridge = CvBridge()
       # Subscribe to the camera image (with ROS_NAMESPACE set, this will be /abot-01/image_raw)
       self.create_subscription(
           Image,
           'image_raw',
           self.image_cb,
           10
       )
       # Publish filtered mask (mono8)
       self.pub_filtered = self.create_publisher(
           Image,
           'image_filtered',
           10
       )
       self.get_logger().info('ColorFilter node started (target = BLUE)')
   def image_cb(self, msg: Image):
       # Convert to OpenCV BGR8
       frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
       # Convert to HSV
       hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
       # Simple blue thresholds (you can tweak later)
       lower_blue = np.array([100, 120, 50])
       upper_blue = np.array([140, 255, 255])
       mask = cv2.inRange(hsv, lower_blue, upper_blue)
       # Optional: a bit of cleanup
       kernel = np.ones((5, 5), np.uint8)
       mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
       mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
       # Publish mono8 mask
       msg_out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
       msg_out.header = msg.header  # keep timestamps
       self.pub_filtered.publish(msg_out)
def main(args=None):
   rclpy.init(args=args)
   node = ColorFilter()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
if __name__ == '__main__':
   main()
