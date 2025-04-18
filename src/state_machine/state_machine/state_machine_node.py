#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String,Float64MultiArray,Bool
from custom_msgs.msg import StateInfo,ButtonCmd,PoseIncrement,InterfaceMultipleMotors,InterfaceSingleMotor


def set_home(set_home_publisher):
    msg = Bool()
    msg.data = True
    set_home_publisher.publish(msg)

def position_cmd(publisher):
    msg = Float64MultiArray()
    msg.data =[209.0,0.0,0.0]
    publisher.publish(msg)

class InitializeState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2", "outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Initialize")
        init_buttons = blackboard["button_cmd"]["init_button"]
        motion_finished = blackboard["motion_finished"]
        init_finished = blackboard["init_finished"]
        motor_ok = blackboard["motor_ok"]

        print("init_buttons:",init_buttons)
        print("motion_finished:",motion_finished)
        print("motor_state:",blackboard["motor_ok"])
        set_home_publisher = blackboard["set_home_publisher"]

        if motor_ok:
            if init_finished:
                blackboard["state_info"]["initialize"] = True
                return "outcome1" #Go To Idle state
            else:
                if motion_finished:  #Ready to move
                    if init_buttons:
                        set_home(set_home_publisher) #move to home
                        return "outcome3"
                    else:
                        return "outcome3"
                else:                   #still tracking
                    return "outcome3"
        else:
            print("error")
            return "outcome3"
class IdleState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3","outcome4"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Idle")
        battery_line_button = blackboard["button_cmd"]["battery_line_button"]
        blackboard["state_info"]["idle"] = True
        motor_ok = blackboard["motor_ok"]

        if motor_ok:
            if blackboard["state_info"]["batterypicker"]: #already changed to picker state
                return "outcome1"
            elif blackboard["state_info"]["batteryassembler"]: #already changed to asselbler state
                return "outcome2"
            else:
                if battery_line_button:
                    return "outcome1"  #run battery picker state
                else:
                    return "outcome4"
        else:
            print("error")
            return "outcome4"
        
class BatteryPickerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BatteryGripper")
        motion_finished = blackboard["motion_finished"]
        position_cmd_publisher = blackboard["position_cmd_publisher"]
        cabinet_line_button = blackboard["button_cmd"]["cabinet_line_button"]
        motor_ok = blackboard["motor_ok"]

        blackboard["state_info"]["idle"] = False
        blackboard["state_info"]["batterypicker"] = True
        
        if motor_ok:
            if cabinet_line_button:
                blackboard["state_info"]["batterypicker"] = False
                blackboard["state_info"]["batteryassembler"] = True
                return "outcome1" #run battery assembler state
            else:
                if motion_finished:
                    position_cmd(position_cmd_publisher) #publish position cmd
                    print("123")
                    return "outcome3"
                else:
                    return "outcome3"
        else:
            print("error")
            return "outcome3"
            
class BatteryAssemblerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BatteryAssembler")
        motion_finished = blackboard["motion_finished"]
        position_cmd_publisher = blackboard["position_cmd_publisher"]
        cabinet_line_button = blackboard["button_cmd"]["cabinet_line_button"]
        motor_ok = blackboard["motor_ok"]

        if motor_ok:
            if cabinet_line_button:
                if motion_finished:
                    position_cmd(position_cmd_publisher) #publish position cmd
                    print("123")
                    return "outcome3"
                else:
                    return "outcome3"
            else:    #cabine off assemble finished
                blackboard["state_info"]["batteryassembler"] = False  
                return "outcome1"  #return to idle state
        else:
            print("error")
            return "outcome3"

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
        self.position_cmd_pub = self.create_publisher(Float64MultiArray,'position_cmd', 10)

        #init motor_state_info
        self.motor_ok = False
        self.init_motors_info()

        #open the blackboard for shareing data in the FSM
        self.blackboard = Blackboard()

        #share device state to FSM
        self.blackboard["motor_ok"] = self.motor_ok

        #store publisher in blackboard 
        self.blackboard["set_home_publisher"] = self.set_home_pub
        self.blackboard["position_cmd_publisher"] = self.position_cmd_pub
        
        #store finished data in blackboard
        self.blackboard["init_finished"] = False
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
                "outcome1": "Idle",
                "outcome2": "Error",
                "outcome3": "Stop",
            },
        )

        self.sm.add_state(
            "Idle",
            IdleState(),
            transitions={
                "outcome1": "BatteryPicker",
                "outcome2": "BatteryAssembler",
                "outcome3": "Error",
                "outcome4": "Stop",
            },
        )

        self.sm.add_state(
            "BatteryPicker",
            BatteryPickerState(),
            transitions={
                "outcome1": "BatteryAssembler",
                "outcome2": "Error",
                "outcome3": "Stop",
            },
        )

        self.sm.add_state(
            "BatteryAssembler",
            BatteryAssemblerState(),
            transitions={
                "outcome1": "Idle",
                "outcome2": "Error",
                "outcome3": "Stop"
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

    def init_motors_info(self):
        self.motors_info = InterfaceMultipleMotors
        self.motors_info.quantity = 3
        self.motors_info.motor_info = [InterfaceSingleMotor() for _ in range(self.motors_info.quantity)]
        self.motors_info.motor_info[0].id = 1
        self.motors_info.motor_info[0].device_state = True
        self.motors_info.motor_info[1].id = 2
        self.motors_info.motor_info[1].device_state = True
        self.motors_info.motor_info[2].id = 3
        self.motors_info.motor_info[2].device_state = True

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
        self.blackboard["motion_finished"] = msg.data  #False = the motor is moving ,can't publish any cmd to motion_control_node

    def init_finished_callback(self,msg:Bool):
        self.blackboard["init_finished"] = msg.data

    def pub_state_info(self,state_info):
        msg = StateInfo()
        msg.initialize = state_info["initialize"]
        msg.idle = state_info["idle"]
        msg.batterypicker = state_info["batterypicker"]
        msg.batteryassembler = state_info["batteryassembler"]
        msg.error = state_info["error"]
        msg.troubleshotting = state_info["troubleshotting"]
        self.state_info_pub.publish(msg)

    def check_device_state(self):
        #check motor:
        for i in range(self.motors_info.quantity):
            if self.motors_info.motor_info[i].device_state:
                all_ok = True
                print(f"M{i+1} is ok")
            else:
                print(f"M{i+1} is failed")
                all_ok = False
        return all_ok
            
    def update_fsm(self):
        self.motor_ok = self.check_device_state()
        self.blackboard["motor_ok"] = self.motor_ok
        print("motor_state:",self.motor_ok)

        print("update_fsm")
        try:
            outcome = self.sm(blackboard=self.blackboard)
            yasmin.YASMIN_LOG_INFO(f"FSM current state: {outcome}")
        except Exception as e:
            self.get_logger().error(f"FSM execution error: {str(e)}")
        #publish FSM state to gui_node
        self.pub_state_info(self.blackboard["state_info"])

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