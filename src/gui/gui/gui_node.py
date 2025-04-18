from custom_msgs.msg import StateInfo, ButtonCmd,PoseIncrement,InterfaceSingleMotor,InterfaceMultipleMotors
from std_msgs.msg import String
from rclpy.node import Node
import rclpy

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Publishers
        self.button_cmd_publisher = self.create_publisher(ButtonCmd, '/button_cmd', 10)
        self.motors_info_publisher = self.create_publisher(InterfaceMultipleMotors,'/multi_motor_info',10)  #fake

        #Subscribers
        self.state_info_subscriber = self.create_subscription(StateInfo, '/state_info', self.state_info_callback ,10)

        self.button_cmd = self.init_button()
        self.button_cmd_publisher.publish(self.button_cmd)
        self.get_logger().info("Initial button_cmd published.")

        # Initialize state
        self.state_info = self.init_state()

        #init motor state
        self.motors_info = self.init_motors_info()

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)  #20Hz
        self.count = 0

    def init_button(self):
        """Initializes the default button cmd."""
        button_cmd_msg = ButtonCmd()
        button_cmd_msg.init_button = False
        button_cmd_msg.battery_line_button = False
        button_cmd_msg.cabinet_line_button = False
        button_cmd_msg.manual_button = False
        button_cmd_msg.gripper_button = False

        return button_cmd_msg

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
    
    def state_info_callback(self,msg:StateInfo):
        self.state_info.initialize = msg.initialize  
        self.state_info.idle = msg.idle  
        self.state_info.batterypicker = msg.batterypicker  
        self.state_info.batteryassembler = msg.batteryassembler
        self.state_info.error = msg.error  
        self.state_info.troubleshotting = msg.troubleshotting  

        print(self.state_info)

    def timer_callback(self):
        """This runs at 20Hz."""
        # self.get_logger().info("UI Node is refreshing...")

        # Publish test ButtonCmd message
        self.button_cmd.init_button = True

        #maunally change butoon state when the state is idle
        if self.state_info.idle:
            self.button_cmd.battery_line_button = True

        if self.count >= 100:
            self.motors_info.motor_info[0].fb_position = 345.0
            self.motors_info.motor_info[1].fb_position = 345.0
            if self.count >= 300:
                self.motors_info.motor_info[0].fb_position = 164.5
                self.motors_info.motor_info[1].fb_position = 253.5
                self.motors_info.motor_info[2].fb_position = 0.0
                self.button_cmd.cabinet_line_button = True

        # #maunally change butoon state check when the state is battery picker and motion finished
        # if self.state_info.batterypicker:
        #     self.button_cmd.cabinet_line_button = True
        
        self.button_cmd_publisher.publish(self.button_cmd)
        self.motors_info_publisher.publish(self.motors_info) #fake
        self.count+=1

def main(args=None):
    rclpy.init(args=args)
    ui_node = UINode()
    rclpy.spin(ui_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()