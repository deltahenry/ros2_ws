import time
import rclpy
from rclpy.node import Node
import threading

import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String, Float64MultiArray
from custom_msgs.msg import StateInfo, ButtonCmd, PoseIncrement


# ... (all your state class definitions remain the same) ...

class InitializeState(State):
    def __init__(self) -> None:

        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        buttons = blackboard["button_cmd"]["init_button"]
        print(buttons)
        yasmin.YASMIN_LOG_INFO("Executing state Initialize")
        time.sleep(3)  # Simulate work

        if self.counter < 3:
            self.counter += 1
            blackboard["Initialize_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"

class IdleState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2","outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Idle")
        time.sleep(3)  # Simulate work
        return "outcome1"
        
class BatteryPickerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2"])
        align_state = 1 #need to modify

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BatteryGripper")
        time.sleep(3)  # Simulate work
        return "outcome1"
            
class BatteryAssemblerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1","outcome2"])
        align_state = 1 #need to modify

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BatteryAssembler")
        time.sleep(3)  # Simulate work
        return "outcome1"

class ErrorState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Error")
        time.sleep(3)  # Simulate work
        return "outcome1"
    
class TroubleshootingState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Error")
        time.sleep(3)  # Simulate work

        yasmin.YASMIN_LOG_INFO(blackboard["Initialize_str"])
        return "outcome1"
class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine_node")

        # Blackboard and thread lock
        self.blackboard_lock = threading.Lock()
        self.blackboard = Blackboard()

        self.blackboard["button_cmd"] = {
            "init_button": True,
            "battery_line_button": False,
            "cabinet_line_button": False,
            "manual_button": False,
            "gripper_button": False,
        }

        # Create FSM
        self.sm = StateMachine(outcomes=["Stop"])

        self.sm.add_state("Initialize", InitializeState(), transitions={
            "outcome1": "Idle", "outcome2": "Error",
        })
        self.sm.add_state("Idle", IdleState(), transitions={
            "outcome1": "Initialize", "outcome2": "Error", "outcome3": "Stop",
        })
        self.sm.add_state("BatteryPicker", BatteryPickerState(), transitions={
            "outcome1": "BatteryAssembler", "outcome2": "Error",
        })
        self.sm.add_state("BatteryAssembler", BatteryAssemblerState(), transitions={
            "outcome1": "Idle", "outcome2": "Error",
        })
        self.sm.add_state("Error", ErrorState(), transitions={
            "outcome1": "Troubleshotting",
        })
        self.sm.add_state("Troubleshotting", TroubleshootingState(), transitions={
            "outcome1": "Initialize",
        })

        YasminViewerPub("yasmin_demo", self.sm)

        self.update_peroid = self.create_timer(1.0, self.update_ui)

        # ROS2 subscriber
        self.button_cmd_subscriber = self.create_subscription(
            ButtonCmd,
            '/button_cmd',
            self.button_cmd_callback,
            10
        )

        # FSM thread control
        self.fsm_thread = None
        self.fsm_running = False

        # Start FSM thread
        self.start_fsm_thread()

    def update_ui(self):
        print("UI")

    def button_cmd_callback(self, msg: ButtonCmd):
        print("Inside button_cmd callback")
        with self.blackboard_lock:
            self.blackboard["button_cmd"]["init_button"] = msg.init_button
            self.blackboard["button_cmd"]["battery_line_button"] = msg.battery_line_button
            self.blackboard["button_cmd"]["cabinet_line_button"] = msg.cabinet_line_button
            self.blackboard["button_cmd"]["manual_button"] = msg.manual_button
            self.blackboard["button_cmd"]["gripper_button"] = msg.gripper_button

    def start_fsm_thread(self):
        if not self.fsm_running:
            self.fsm_running = True
            self.fsm_thread = threading.Thread(target=self.run_state_machine)
            self.fsm_thread.start()

    def run_state_machine(self):
        try:
            while rclpy.ok():
                with self.blackboard_lock:
                    outcome = self.sm(blackboard=self.blackboard)
                yasmin.YASMIN_LOG_INFO(f"FSM outcome: {outcome}")
                if outcome == "Stop":
                    break
        except Exception as e:
            self.get_logger().error(f"FSM thread crashed: {e}")
        finally:
            self.fsm_running = False


def main():
    rclpy.init()
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
