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

        self.sub = self.create_subscription(Image,'/image_filtered',self.image,10)  #subscribes to image_filtered which allowes us to get the data from the image
        self.pub = self.create_publisher(Int32MultiArray, 'image_info', 10) #publishes to image_info node which allows us to publish the infomation of the image 
        self.bridge = CvBridge() #convers ROS images to OpenCV

    def image(self, image_message):
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8') #sets the image type back to openCV as 8 bit rgb (bgr8)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), cv2.BORDER_DEFAULT)
        ret, thresh = cv2.threshold(blur, 1, 255, cv2.THRESH_BINARY) 
        contours, hierarchies = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #lines 18 -22 basicly outline the image or shape (found on Open CV python)
        object_width = 0

        shape = max(contours, key=cv2.contourArea)  
        x,y,w,h = cv2.boundingRect(shape)
        object_width = w                        #width of the object
        center = x + (w/2)               #center of the object in the image
        y, x = gray.shape               #get image height and width
        offset = center - (x/2)         # image from center

        #lines 28 to 35 will basicly do the math. cv2.boundingrect makes a rectangle about the image and gives x,y,w,h -> used to calculate the width of object and how far from center (ofset) 
        msg_out = Int32MultiArray()
        msg_out.data = [int(center), int(object_width)]
        self.pub.publish(msg_out)

        print(f"offse from center={offset}, object width={object_width}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageInfo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
