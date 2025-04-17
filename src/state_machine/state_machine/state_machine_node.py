#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String,Float64MultiArray,Bool
from custom_msgs.msg import StateInfo, ButtonCmd,PoseIncrement,Finished


def set_home(publisher):
    msg = Bool()
    msg.data = True
    publisher.publish(msg)

def state_pub(publisher,state_info):
    msg = StateInfo()
    msg.initialize = state_info
    publisher.publish(msg)
class InitializeState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Initialize")
        init_buttons = blackboard["button_cmd"]["init_button"]
        motion_finished = blackboard["motion_finished"]
        init_finished = blackboard["init_finished"]

        print("init_buttons:",init_buttons)
        print("motion_finished:",motion_finished)

        set_home_publisher = blackboard["set_home_publisher"]
        state_publisher = blackboard["state_publisher"]

        if init_finished:
            blackboard["state_info"]["initialize"] = True
            state_pub(state_publisher,blackboard["state_info"]["initialize"])
            return "outcome2" #Go To Idle state
        else:
            if motion_finished:  #Ready to move
                if init_buttons:
                    set_home(set_home_publisher)
                    return "outcome1"
                else:
                    return "outcome1"
            else:                   #still tracking
                return "outcome1"

class IdleState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Idle")
        return "outcome3"
        
class BatteryPickerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BatteryGripper")
        return "outcome1"
            
class BatteryAssemblerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BatteryAssembler")
        return "outcome1"

class ErrorState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Error")
        return "outcome1"
    
class TroubleshootingState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Error")

        yasmin.YASMIN_LOG_INFO(blackboard["Initialize_str"])
        return "outcome1"

class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine_node")

        #subscriber
        self.button_cmd_sub = self.create_subscription(ButtonCmd,'/button_cmd',self.button_cmd_callback,10)
        self.motion_finished_sub = self.create_subscription(Bool,'/motion_finished',self.motion_finished_callback,10)
        self.init_finished_sub = self.create_subscription(Bool,'/init_finished',self.init_finished_callback,10)

        #publiher
        self.set_home_pub = self.create_publisher(Bool, '/set_home_cmd', 10) #to_motion_control
        self.state_info_pub = self.create_publisher(StateInfo, '/state_info', 10) #to_gui_node
        
        #open the blackboard
        self.blackboard = Blackboard()

        #store publisher in blackboard 
        self.blackboard["set_home_publisher"] = self.set_home_pub
        self.blackboard["state_publisher"] = self.state_info_pub
        
        #store init_finished data in blackboard
        self.blackboard["init_finished"] = False

        #store motion_finished data in blackboard
        self.blackboard["motion_finished"] = False

        #init button_cmd in blackboard
        self.blackboard["button_cmd"]={
            "init_button":False,
            "battery_line_button":False,
            "cabinet_line_button":False,
            "manual_button":False,
            "gripper_button":False,
        }

        #init state_info in blackboard
        self.blackboard["state_info"]={
            "initialize":False,      
            "idle":False,
            "batterypicker":False,
            "batteryassembler":False,
            "error":False,
            "troubleshotting":False,

        }

        # Create a finite state machine (FSM)
        self.sm = StateMachine(outcomes=["Stop"])

        # Add states to the FSM
        self.sm.add_state(
            "Initialize",
            InitializeState(),  # No need to pass publisher anymore
            transitions={
                "outcome1": "Stop",
                "outcome2": "Idle",
            },
        )

        self.sm.add_state(
            "Idle",
            IdleState(),
            transitions={
                "outcome1": "BatteryPicker",
                "outcome2": "Error",
                "outcome3": "Stop",
            },
        )

        self.sm.add_state(
            "BatteryPicker",
            BatteryPickerState(),
            transitions={
                "outcome1": "BatteryAssembler",
                "outcome2": "Error",
            },
        )

        self.sm.add_state(
            "BatteryAssembler",
            BatteryAssemblerState(),
            transitions={
                "outcome1": "Idle",
                "outcome2": "Error",
            },
        )

        self.sm.add_state(
            "Error",
            ErrorState(),
            transitions={
                "outcome1": "Troubleshotting",
            },
        )

        self.sm.add_state(
            "Troubleshotting",
            TroubleshootingState(),
            transitions={
                "outcome1": "Initialize",
            },
        )

        # Publish FSM information for visualization
        YasminViewerPub("yasmin_demo", self.sm)

        # Timer to periodically publish data to ui_node
        self.timer = self.create_timer(1.0, self.update_fsm)


    def button_cmd_callback(self,msg:ButtonCmd):
        print("inside bmc callback")

        self.blackboard["button_cmd"]={
            "init_button":msg.init_button,
            "battery_line_button":msg.battery_line_button,
            "cabinet_line_button":msg.cabinet_line_button,
            "manual_button":msg.manual_button,
            "gripper_button":msg.gripper_button,
        }

    def motion_finished_callback(self,msg:Bool):
        self.blackboard["motion_finished"] = msg.data

    def init_finished_callback(self,msg:Bool):
        self.blackboard["init_finished"] = msg.data

    def update_fsm(self):
        print("update_fsm")
        try:
            outcome = self.sm(blackboard=self.blackboard)
            yasmin.YASMIN_LOG_INFO(f"FSM current state: {outcome}")
        except Exception as e:
            self.get_logger().error(f"FSM execution error: {str(e)}")

def main():
    rclpy.init()

    # Initialize the ROS 2 node
    node = StateMachineNode()

    # Spin to keep the node alive
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
