import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Mode(Node):
   def __init__(self):
       super().__init__('mode_node')
       self.pub = self.create_publisher(String, '/mode', 10)
       self.get_logger().info(
           'Mode node ready. Type: t=teleop, a=auto, e=estop, q=quit'
       )
   def run(self):
       try:
           while rclpy.ok():
               key = input('[t]eleop  [a]uto  [e]stop  [q]uit: ').strip().lower()
               if not key:
                   continue
               if key[0] == 'q':
                   self.get_logger().info('Quitting mode node.')
                   break
               msg = String()
               msg.data = key[0]
               self.pub.publish(msg)
       except KeyboardInterrupt:
           pass
def main(args=None):
   rclpy.init(args=args)
   node = Mode()
   node.run()
   node.destroy_node()
   rclpy.shutdown()
if __name__ == '__main__':
   main()
