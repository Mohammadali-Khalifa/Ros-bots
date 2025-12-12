import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFilter(Node):
    def __init__(self):
        super().__init__('color_filter')
        self.pub = self.create_publisher(Image, '/image_filtered', 10) #publishes to image_filter used by color_filter
        self.sub = self.create_subscription(Image, '/image_raw', self.filter, 10) #subscribes to image_filter used by color_filter
        self.bridge = CvBridge()
        self.declare_parameter('target_color', 'blue') #sets the deault to blue
        self.declare_parameter('erode_dilate_iters', 2) #sets the dilate and iters to 2

    def filter(self, image_message): # convert ROS image to OpenCV
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        color = self.get_parameter('target_color').value
        if color == 'blue':
            mask = cv2.inRange(hsv, (90, 80, 40), (140, 255, 255))
        elif color == 'pink':
            mask = cv2.inRange(hsv, (140, 40, 40), (180,255,255))
        elif color == 'white':
            mask = cv2.inRange(hsv, (0,0,180), (40,150,255))
        else:
            mask = cv2.inRange(hsv, (0,0,0), (180,255,255))

        #lines 18 to 29 basicly do the filtering and changing the HSV numbers of low and high can filter more or less (creates a bit mask for the color)

        
        kernel = np.ones((7,7), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        result = cv2.bitwise_and(img, img, mask=mask)
        ros_img = self.bridge.cv2_to_imgmsg(result, "bgr8")
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
