import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#color filtering node that subscribes to camera image, filters color in HSV, published binary mask of detected color
class ColorFilter(Node):
    def __init__(self):
        super().__init__('color_filter')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.filter, 10) #subscribes to image_filter used by color_filter
        self.pub = self.create_publisher(Image, 'image_filtered', 10) #publishes to image_filter used by color_filter
    def filter(self, image_message): 
        bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8') # convert ROS image to OpenCV BGR image
        bgr = cv2.rotate(bgr, cv2.ROTATE_180) #rotate image 180 degrees to correct camera orientation
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV) #convert BGR to HSV
        mask = cv2.inRange(hsv, (140, 40, 40), (180,255,255)) #create binary mask for target color in HSV range
        kernel = np.ones((7,7), np.uint8) #create kernel for morphological operations
        mask = cv2.erode(mask, kernel, iterations=1) #erosion and dilation to remove noise
        mask = cv2.dilate(mask, kernel, iterations=2)
        #lines 14 to 20 basicly do the filtering and changing the HSV numbers of low and high can filter more or less (creates a bit mask for the color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #find contours in mask
        clean_mask = np.zeros_like(mask) #create new mask 
        if contours:
            largest = max(contours, key=cv2.contourArea) #select largest countour
            min_area = 500 #minimum area threshold to reject detecting small contours
            if cv2.contourArea(largest) > min_area:
                cv2.drawContours(clean_mask, [largest], -1, 255, thickness=-1) #draw largest countour as a filled shape
            else:
                clean_mask = mask #countour too small keep original
        else:
            clean_mask = mask #none found, keep original
        ros_img = self.bridge.cv2_to_imgmsg(clean_mask, "mono8")
        self.pub.publish(ros_img) #publish filterred image
        #lines 32 to 34 are form slides 24-25 of computer vision and then it publshes the images
def main(args=None):
    rclpy.init(args=args)
    node = ColorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
