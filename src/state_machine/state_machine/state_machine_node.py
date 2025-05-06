#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String,Float32MultiArray,Bool,Int32
from custom_msgs.msg import StateInfo,ButtonCmd,PoseIncrement,InterfaceMultipleMotors,InterfaceSingleMotor


def set_home(set_home_publisher,set_home_data):
    msg = Bool()
    msg.data = set_home_data
    set_home_publisher.publish(msg)

def position_cmd(publisher,x_cmd,y_cmd,yaw_cmd):
    msg = Float32MultiArray()
    msg.data =[x_cmd,y_cmd,yaw_cmd]
    print("X_cmd",x_cmd)
    print("Y_cmd",y_cmd)
    print("Yaw_cmd",yaw_cmd)
    publisher.publish(msg)

class InitializeState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2", "outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state Initialize")
        init_buttons = blackboard["button_cmd"]["init_button"]
        motion_finished = blackboard["motion_finished"]
        init_finished = blackboard["init_finished"]
        motor_ok = blackboard["motor_ok"]

        # print("init_buttons:",init_buttons)
        # print("motion_finished:",motion_finished)
        # print("motor_state:",blackboard["motor_ok"])
        set_home_publisher = blackboard["set_home_publisher"]

        if motor_ok:
            if init_finished:
                blackboard["state_info"]["idle"] = True
                blackboard["set_home"] = False
                set_home(set_home_publisher,blackboard["set_home"]) #dot't move to home
                return "outcome1" #Go To Idle state
        
            else:
                if motion_finished:  #Ready to move
                    if init_buttons:
                        blackboard["state_info"]["initialize"] = True
                        blackboard["state_info"]["troubleshotting"] = False
                        blackboard["set_home"] = True
                        set_home(set_home_publisher,blackboard["set_home"]) #move to home
                        return "outcome3"
                    else:
                        return "outcome3"
                else:                   #still tracking
                    return "outcome3"
        else:
            print("error")
            blackboard["state_info"]["initialize"] = False
            blackboard["state_info"]["error"] = True
            return "outcome2"
class IdleState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3","outcome4"])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state Idle")
        battery_line_button = blackboard["button_cmd"]["battery_line_button"]
        motor_ok = blackboard["motor_ok"]

        if motor_ok:
            if blackboard["state_info"]["batterypicker"]: #already changed to picker state
                blackboard["state_info"]["idle"] = False
                return "outcome1"
            elif blackboard["state_info"]["batteryassembler"]: #already changed to asselbler state
                blackboard["state_info"]["idle"] = False
                return "outcome2"
            else:
                if battery_line_button:
                    blackboard["state_info"]["idle"] = False
                    blackboard["state_info"]["batterypicker"] = True
                    return "outcome1"  #run battery picker state
                else:
                    return "outcome4"
        else:
            print("error")
            blackboard["state_info"]["initialize"] = False
            blackboard["state_info"]["idle"] = False
            blackboard["state_info"]["error"] = True
            return "outcome2"
        
class BatteryPickerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state BatteryGripper")
        motion_finished = blackboard["motion_finished"]
        position_cmd_publisher = blackboard["position_cmd_publisher"]
        cabinet_line_button = blackboard["button_cmd"]["cabinet_line_button"]
        motor_ok = blackboard["motor_ok"]
        pos_cmd = [blackboard["pos_cmd"]["x"] ,blackboard["pos_cmd"]["y"] ,blackboard["pos_cmd"]["yaw"]]
        pos_cmd_past = [blackboard["pos_cmd"]["x_past"] ,blackboard["pos_cmd"]["y_past"] ,blackboard["pos_cmd"]["yaw_past"]]
        
        if motor_ok:
            if cabinet_line_button:
                blackboard["state_info"]["batterypicker"] = False
                blackboard["state_info"]["batteryassembler"] = True
                return "outcome1" #run battery assembler state
            else:
                if motion_finished:
                    if any(a != b for a,b in zip(pos_cmd,pos_cmd_past)):
                        position_cmd(position_cmd_publisher,pos_cmd[0],pos_cmd[1],pos_cmd[2]) #publish position cmd
                        blackboard["pos_cmd"]["x_past"] = pos_cmd[0]
                        blackboard["pos_cmd"]["y_past"] = pos_cmd[1]
                        blackboard["pos_cmd"]["yaw_past"] = pos_cmd[2]
                    return "outcome3"
                else:
                    return "outcome3"
        else:
            print("error")
            blackboard["state_info"]["initialize"] = False
            blackboard["state_info"]["batterypicker"] = False
            blackboard["state_info"]["error"] = True
            return "outcome2"
            
class BatteryAssemblerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state BatteryAssembler")
        motion_finished = blackboard["motion_finished"]
        position_cmd_publisher = blackboard["position_cmd_publisher"]
        cabinet_line_button = blackboard["button_cmd"]["cabinet_line_button"]
        motor_ok = blackboard["motor_ok"]
        pos_cmd = [blackboard["pos_cmd"]["x"] ,blackboard["pos_cmd"]["y"] ,blackboard["pos_cmd"]["yaw"]]
        pos_cmd_past = [blackboard["pos_cmd"]["x_past"] ,blackboard["pos_cmd"]["y_past"] ,blackboard["pos_cmd"]["yaw_past"]]

        if motor_ok:
            if not cabinet_line_button:    #cabinet off assemble finished
                blackboard["state_info"]["batteryassembler"] = False  
                blackboard["state_info"]["idle"] = True
                return "outcome1"  #return to idle state
            else:
                if motion_finished:
                    if any(a != b for a,b in zip(pos_cmd,pos_cmd_past)):
                        position_cmd(position_cmd_publisher,pos_cmd[0],pos_cmd[1],pos_cmd[2]) #publish position cmd
                        blackboard["pos_cmd"]["x_past"] = pos_cmd[0]
                        blackboard["pos_cmd"]["y_past"] = pos_cmd[1]
                        blackboard["pos_cmd"]["yaw_past"] = pos_cmd[2]
                    return "outcome3"
                else:
                    return "outcome3"
        else:
            print("error")
            blackboard["state_info"]["initialize"] = False
            blackboard["state_info"]["batteryassembler"] = False
            blackboard["state_info"]["error"] = True
            return "outcome2"

class ErrorState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state Error")
        manual_button = blackboard["button_cmd"]["manual_button"]

        if manual_button:
            blackboard["state_info"]["error"] = False
            blackboard["state_info"]["troubleshotting"] = True
            return "outcome1"
        else:
            return "outcome2"
    
class TroubleshootingState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Error")
        init_buttons = blackboard["button_cmd"]["init_button"]

        if init_buttons:
            blackboard["state_info"]["initialize"] = True
            return "outcome1"
        else:
            return "outcome2"
class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine_node")

        #subscriber
        self.button_cmd_sub = self.create_subscription(ButtonCmd,'/button_cmd',self.button_cmd_callback,10)
        self.motion_finished_sub = self.create_subscription(Bool,'/motion_finished',self.motion_finished_callback,10)
        self.init_finished_sub = self.create_subscription(Bool,'/init_finished',self.init_finished_callback,10)
        self.pose_increment_sub = self.create_subscription(PoseIncrement,'pose_increment',self.pose_increment_callback,10)
        self.esm_info_sub = self.create_subscription(Int32,'/esm_alarm',self.esm_info_callback,10)

        #publiher
        self.set_home_pub = self.create_publisher(Bool, '/set_home_cmd', 10) #to_motion_control
        self.state_info_pub = self.create_publisher(StateInfo, '/state_info', 10) #to_gui_node
        self.position_cmd_pub = self.create_publisher(Float32MultiArray,'position_cmd', 10)
        self.servo_switch_pub = self.create_publisher(Bool, '/servo_switch', 10) #to_motor node

        #init motor_state_info
        self.esm_info = -99
        self.motor_ok = False
        self.init_motors_info()

        #init pos_cmd
        self.init_pos_cmd()

        #open the blackboard for shareing data in the FSM
        self.blackboard = Blackboard()

        #share pos_cmd to blackboard
        self.blackboard["pos_cmd"] = {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "x_past": 0.0,
            "y_past": 0.0,
            "yaw_past": 0.0
        }

        self.blackboard["set_home"] = False

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
                "outcome2": "Stop",
            },
        )

        self.sm.add_state(
            "Troubleshotting",
            TroubleshootingState(),
            transitions={
                "outcome1": "Initialize",
                "outcome2": "Stop",
            },
        )

        # Publish FSM information for visualization
        YasminViewerPub("yasmin_demo", self.sm)

        # Timer to periodically publish data to ui_node
        self.timer = self.create_timer(0.5, self.update_fsm)

    def esm_info_callback(self,msg:Int32):
        # print("esm:",msg.data)
        self.esm_info = msg.data

    def button_cmd_callback(self,msg:ButtonCmd):
        # print("inside bmc callback")

        self.blackboard["button_cmd"]={
            "init_button":msg.init_button,
            "battery_line_button":msg.battery_line_button,
            "cabinet_line_button":msg.cabinet_line_button,
            "manual_button":msg.manual_button,
            "gripper_button":msg.gripper_button,
        }

    def pose_increment_callback(self,msg:PoseIncrement):
        if self.blackboard["motion_finished"]:
            base_increments = {
                0: 1.0,        # x
                1: 5.0,        # y
                2: 0.017       # yaw (1 degree in radians)
            }
            # Define a scaling factor for step size
            step_scale = {
                1: 1,
                2: 5,
                3: 10,
                -1: -1,
                -2: -5,
                -3: -10
            }
            # Get the scale and increment for current msg
            scale = step_scale.get(msg.step, 0)
            base = base_increments.get(msg.axis, 0)
            delta = scale * base

            # Apply the change
            if msg.axis == 0:
                self.x += delta
            elif msg.axis == 1:
                self.y += delta
            elif msg.axis == 2:
                self.yaw += delta
            else:
                print("Invalid axis")

            self.blackboard["pos_cmd"]["x"] = self.x
            self.blackboard["pos_cmd"]["y"] = self.y
            self.blackboard["pos_cmd"]["yaw"] = self.yaw


        else:
            print("device is running")
    
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

    def init_motors_info(self):
        self.motors_info = InterfaceMultipleMotors()
        self.motors_info.quantity = 3
        self.motors_info.motor_info = [InterfaceSingleMotor() for _ in range(self.motors_info.quantity)]
        self.motors_info.motor_info[0].id = 1
        self.motors_info.motor_info[0].servo_state = True
        self.motors_info.motor_info[1].id = 2
        self.motors_info.motor_info[1].servo_state = True
        self.motors_info.motor_info[2].id = 3
        self.motors_info.motor_info[2].servo_state = True

    def init_pos_cmd(self):
        self.x = 345.0
        self.y = 0.0
        self.yaw = 0.0

    def check_device_state(self):
        #check motor:
        # for i in range(self.motors_info.quantity):
        #     if self.motors_info.motor_info[i].servo_state:
        #         all_ok = True
        #         print(f"M{i+1} is ok")
        #     else:
        #         print(f"M{i+1} is failed")
        #         all_ok = False
        if self.esm_info == 0:
            all_ok = True
        else:
            all_ok = False

        return all_ok
            
    def update_fsm(self):
        self.motor_ok = self.check_device_state()
        self.blackboard["motor_ok"] = self.motor_ok

        if not self.blackboard["init_finished"] and not self.motor_ok: 
            motor_power = Bool()
            motor_power.data = True
            self.servo_switch_pub.publish(motor_power)
            print("motor_state:",self.motor_ok)
        else:
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