from custom_msgs.msg import StateInfo, ButtonCmd,PoseIncrement,InterfaceSingleMotor,InterfaceMultipleMotors
from std_msgs.msg import String
from rclpy.node import Node
import rclpy

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Publishers
        self.button_cmd_publisher = self.create_publisher(ButtonCmd, '/button_cmd', 10)
        self.a_publisher = self.create_publisher(InterfaceMultipleMotors,'/multi_motor_info',10)

        #Subscribers
        self.state_info_subscriber = self.create_publisher(StateInfo, '/state_info', 10)

        # Initialize button
        self.init_button()
        # Initialize state
        self.init_state()

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

    def init_button(self):
        """Initializes the default button cmd."""
        button_cmd_msg = ButtonCmd()
        button_cmd_msg.init_button = False
        button_cmd_msg.battery_line_button = False
        button_cmd_msg.cabinet_line_button = False
        button_cmd_msg.manual_button = False
        button_cmd_msg.gripper_button = False

        self.button_cmd_publisher.publish(button_cmd_msg)
        self.get_logger().info("Initial button_cmd published.")

    def init_state(self):
        """Initializes the default state."""
        state_info_msg = StateInfo()
        state_info_msg.initialize = False
        state_info_msg.idle = False
        state_info_msg.batterypicker = False
        state_info_msg.batteryassembler = False
        state_info_msg.error = False
        state_info_msg.troubleshotting = False

    def timer_callback(self):
        """This runs at 20Hz."""
        # self.get_logger().info("UI Node is refreshing...")

        # Publish test ButtonCmd message
        msg = ButtonCmd()
        msg.init_button = True
        self.button_cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ui_node = UINode()
    rclpy.spin(ui_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
