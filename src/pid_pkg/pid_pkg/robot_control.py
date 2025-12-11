import rclpy
from rclpy.node import Node
from odometry_helper_msg.msg import DistWheel
import math 
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.kp = 0.6 #proportional
        self.ki = 0.05 #integral
        self.kd = 7.5 #derivative

        self.dt = 0.1 #dt for derivative
        self.Desired = 0.0 #error
        self.I = 0.0 #vrable to save intergral
        self.Actual = 0.0 # actual

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.error = self.create_subscription(Float32, 'error', self.callback, 10) 
        self.start = self.create_client(SetBool, "controller_ready")
        self.publisher_ = self.create_publisher(Float32, 'control_input', 10)
        self.Flag = self.create_timer(0.5, self.start_system)

    def callback(self, msg: Float32):
        self.Desired = float(msg.data) #saves the incoming error value in a varable desired

    def start_system(self):
        while not self.start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Controler: Waiting to start')
        req = SetBool.Request()
        req.data = True
        self.start.call_async(req)

    # lines 30 to 35 sends the controller_ready signal to start the graph and dynamics


    def timer_callback(self):
        P = self.kp * self.Desired
        self.I = self.I + (self.Desired * self.dt)
        I = self.ki * self.I
        D = self.kd * (self.Desired - self.Actual) / self.dt
        self.Actual = self.Desired
        PID = P + I + D

        msg = Float32()
        msg.data = float(PID)
        self.publisher_.publish(msg)

    #lines 40-50 is the math for the PID per slide 54 of PPT 7. It does the math and then publishes it

def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()               
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
  
