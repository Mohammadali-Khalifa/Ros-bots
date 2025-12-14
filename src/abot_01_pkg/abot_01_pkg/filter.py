import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class ColorFilter(Node):
    def __init__(self):
        super().__init__('color_filter')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.filter, 10) #subscribes to image_filter used by color_filter
        self.pub = self.create_publisher(Image, 'image_filtered', 10) #publishes to image_filter used by color_filter
        
    def filter(self, image_message): # convert ROS image to OpenCV
        bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (95, 120, 60), (130, 255, 255))
        #lines 18 to 29 basicly do the filtering and changing the HSV numbers of low and high can filter more or less (creates a bit mask for the color)
        kernel = np.ones((7,7), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        clean_mask = np.zeros_like(mask)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            min_area = 500
            if cv2.contourArea(largest) > min_area:
                cv2.drawContours(clean_mask, [largest], -1, 255, thickness=-1)
            else:
                clean_mask = mask
        else:
            clean_mask = mask
            
        ros_img = self.bridge.cv2_to_imgmsg(clean_mask, "mono8")
        self.pub.publish(ros_img)
        #lines 34 to 39 are form slides 24-25 of computer vision and then it publshes the images

def main(args=None):
    rclpy.init(args=args)
    node = ColorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
