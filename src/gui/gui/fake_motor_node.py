from custom_msgs.msg import StateInfo, ButtonCmd,PoseIncrement,InterfaceSingleMotor,InterfaceMultipleMotors
from std_msgs.msg import String,Bool,Float64MultiArray
from rclpy.node import Node
import rclpy

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Publishers
        self.motors_info_publisher = self.create_publisher(InterfaceMultipleMotors,'/multi_motor_info',10)  #fake

        #Subscribers
        self.state_info_subscriber = self.create_subscription(StateInfo, '/state_info', self.state_info_callback ,10)
        self.set_home_subscriber = self.create_subscription(Bool, '/set_home_cmd',self.set_home_callback, 10) #to_motion_control
        self.motor_pub = self.create_subscription(Float64MultiArray, '/motor_position_ref',self.motor_cmd_callback, 10) #motor_node

        self.set_home = False

        # Initialize state
        self.state_info = self.init_state()

        #init motor state
        self.motors_info = self.init_motors_info()

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)  #20Hz
        self.pub_count = 0

    def init_state(self):
        """Initializes the default state."""
        state_info_msg = StateInfo()
        state_info_msg.initialize = False
        state_info_msg.idle = False
        state_info_msg.batterypicker = False
        state_info_msg.batteryassembler = False
        state_info_msg.error = False
        state_info_msg.troubleshotting = False
        
        return state_info_msg
    
    def init_motors_info(self):
        self.motors_info = InterfaceMultipleMotors()
        self.motors_info.quantity = 3
        self.motors_info.motor_info = [InterfaceSingleMotor() for _ in range(self.motors_info.quantity)]
        self.motors_info.motor_info[0].id = 1
        self.motors_info.motor_info[0].fb_position = 330.0
        self.motors_info.motor_info[1].id = 2
        self.motors_info.motor_info[1].fb_position = 400.0
        self.motors_info.motor_info[2].id = 3
        self.motors_info.motor_info[2].fb_position = 0.0
        return self.motors_info
    
    def motor_cmd_callback(self,msg:Float64MultiArray):
        # print(msg.data[27])
        self.motors_info.motor_info[0].fb_position = msg.data[27]
        self.motors_info.motor_info[1].fb_position = msg.data[28]
        self.motors_info.motor_info[2].fb_position = msg.data[29]

    def set_home_callback(self,msg:Bool):
        self.set_home = msg.data

    def state_info_callback(self,msg:StateInfo):
        self.state_info.initialize = msg.initialize  
        self.state_info.idle = msg.idle  
        self.state_info.batterypicker = msg.batterypicker  
        self.state_info.batteryassembler = msg.batteryassembler
        self.state_info.error = msg.error  
        self.state_info.troubleshotting = msg.troubleshotting  

        # print(self.state_info)

    def timer_callback(self):
        """This runs at 20Hz."""

        if self.pub_count >=100:
            self.motors_info_publisher.publish(self.motors_info) #fake
            self.pub_count = 0

        self.pub_count+=1

def main(args=None):
    rclpy.init(args=args)
    ui_node = UINode()
    rclpy.spin(ui_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()