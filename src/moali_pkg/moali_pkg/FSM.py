import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class FSM_Code(Node):
    def __init__(self):
        super().__init__('FSM')

        self.autonomous = self.create_subscription(Twist, '/auto/cmd_vel', self.auto, 10)           # subscribes to auto/cmd_vel which is used for my move_square.py
        self.teleoperated = self.create_subscription(Twist, '/turtle1/cmd_vel', self.teleop, 10)     # subscribes to teleop which allows the uses of the keyboard input
        self.create_subscription(String, '/mode', self.mode, 10)                                      # subscribes to the mode to select the modes
        self.create_subscription(Bool, '/estop_reset', self.estop, 10)                                 # subscribes to the estop for the reset
        self.publish = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)                          # publishes to the terminal

        self.mode_requested = 'Teleoperated'                                                       
        self.mode_selected  = 'Teleoperated'                                                       
        self.estop_latched  = False                                                                    
        self.last_auto_state = Twist()         
        self.last_teleop_state = Twist()       

        # lines 17 to 21 sets teleoperated as the default and then saves the state mesage

        self.timer = self.create_timer(.1, self.refresh)                                              # refreshes to show every .1 seconds

    def auto(self, msg: Twist):                
        self.last_auto_state = msg
    def teleop(self, msg: Twist):               
        self.last_teleop_state = msg

    #lines 27 to 30 saves te last message used in auto and teleop

    def mode(self, msg: String):                                                                     # checks which mode was selected
        msg = msg.data
        if msg == 'estop':
            self.estop_active()                                                                      # sets estop latch
            self.get_logger().info('Mode = ESTOP ACTIVE')
        elif msg == 'ostop':
            self.mode_requested = 'Obstacle Stop'                                                       # sets mode to ostop
            self.activate_mode()
            self.get_logger().info('Mode = Obstacle Stop')
        elif msg == 'teleop':
            self.mode_requested = 'Teleoperated'                                                        # sets mode to teleoperated
            self.activate_mode()
            self.get_logger().info('Mode = Teleoperated')
        elif msg == 'auto':
            self.mode_requested = 'Autonomous'                                                           # sets mode to autonomous
            self.activate_mode()
            self.get_logger().info('Mode = Autonomous')
        else:
            self.get_logger().warn(f'Unknown mode request: "{msg}"')

    def estop(self, msg: Bool):               
        if msg.data and self.estop_latched:
            self.estop_latched = False
            self.get_logger().info('E-STOP unlatched')
            self.activate_mode()
    def estop_active(self):                    
        self.estop_latched = True
        self.mode_selected = 'ESTOP (Latched)'
        self.publish.publish(Twist())           

    #lines 54 to 62 which checks if the estop is latched or not and if its latched it pauses the turtle until its unlactched


    def activate_mode(self):                   
        if self.estop_latched:
            self.mode_selected = 'ESTOP (Latched)'
        elif self.mode_requested == 'Obstacle Stop':
            self.mode_selected = 'Obstacle Stop'
        else:
            self.mode_selected = self.mode_requested

    def refresh(self):                          
        out = Twist()
        if self.mode_selected == 'Autonomous':
            out = self.last_auto_state
        elif self.mode_selected == 'Teleoperated':
            out = self.last_teleop_state
        self.publish.publish(out)               

def main(args=None):
    rclpy.init(args=args)               
    node = FSM_Code()                    
    rclpy.spin(node)                           
    node.destroy_node()                     
    rclpy.shutdown()                          

if __name__ == '__main__':
    main()                                     
