class ImageInfo(Node):
    def __init__(self):
        super().__init__('image_info')

        self.bridge = CvBridge()

        # IMPORTANT: must match the absolute topic published by filter.py
        self.sub = self.create_subscription(Image, '/image_filtered', self.image_cb, 10)

        self.pub = self.create_publisher(Int32MultiArray, 'image_info', 10)

    def image_cb(self, msg: Image):
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        center_px = -1
        width_px = 0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            center_px = x + (w // 2)
            width_px = w

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
