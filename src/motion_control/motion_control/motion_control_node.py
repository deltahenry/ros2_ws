import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Pose2D
import numpy as np
import math
from custom_msgs.msg import InterfaceMultipleMotors, InterfaceSingleMotor



class MotionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        self.declare_parameter('v_des', 1.0)
        self.declare_parameter('batch_size', 10)

        self.v_des = self.get_parameter('v_des').get_parameter_value().double_value
        self.batch_size = self.get_parameter('batch_size').get_parameter_value().integer_value

        self.current_pose = [0.0, 0.0, 0.0]
        self.current_motor_pos = [0.0, 0.0, 0.0]
        self.last_sent_batch = []

        self.position_queue = []
        self.home_queue = []

        self.has_new_position = False
        self.has_new_home = False

        self.check_home_motion = False
        self.check_position_motion = False

        self.is_idle = True #pub ready to move(in the beginning)

        #subscriber
        self.set_home_sub = self.create_subscription(Bool, '/set_home_cmd', self.set_home_callback, 10)
        self.pos_cmd_sub = self.create_subscription(Float64MultiArray, '/position_cmd', self.position_cmd_callback, 10)
        self.motors_info_sub = self.create_subscription(InterfaceMultipleMotors,'/multi_motor_info',self.motors_info_callback,10)

        #publisher
        self.motor_pub = self.create_publisher(Float64MultiArray, '/motor_position_ref', 10)
        self.motion_finished_pub = self.create_publisher(Bool, '/motion_finished', 10)
        self.init_finished_pub = self.create_publisher(Bool, '/init_finished', 10)

        #init motor_state_info
        self.motor_ok = False
        self.init_motors_info()

        self.timer = self.create_timer(1.0 / 40.0, self.timer_callback)
    
    def init_motors_info(self):
        self.motors_info = InterfaceMultipleMotors()
        self.motors_info.quantity = 3
        self.motors_info.motor_info = [InterfaceSingleMotor() for _ in range(self.motors_info.quantity)]
        self.motors_info.motor_info[0].id = 1
        self.motors_info.motor_info[0].fb_position = 0.0
        self.motors_info.motor_info[1].id = 2
        self.motors_info.motor_info[1].fb_position = 0.0
        self.motors_info.motor_info[2].id = 3
        self.motors_info.motor_info[2].fb_position = 0.0

    def position_cmd_callback(self, msg:Float64MultiArray):
        x_start, y_start, yaw_start = self.current_pose
        x_end, y_end, yaw_end = msg.data[0], msg.data[1], msg.data[2]

        position_trajectory = self.generate_trajectory(x_start, y_start, yaw_start, x_end, y_end, yaw_end)
        position_trajectory = self.pad_trajectory(position_trajectory)
        self.position_queue = position_trajectory.copy()
        self.has_new_position = True

    def set_home_callback(self, msg):
        if msg.data:
            home_position = [0.0, 0.0, 0.0]
            start = self.current_motor_pos
            home_trajectory = self.generate_home_trajectory(start, home_position)
            home_trajectory = self.pad_trajectory(home_trajectory)  #10
            # print("home.tra:",home_trajectory)
            self.home_queue = home_trajectory.copy()
            self.has_new_home = True

    def motors_info_callback(self, msg:InterfaceMultipleMotors):
        # print("NON")
        print(msg.motor_info[0].fb_position)


    def generate_trajectory(self, x_start, y_start, yaw_start, x_end, y_end, yaw_end):
        dx, dy, dyaw = x_end - x_start, y_end - y_start, yaw_end - yaw_start
        dist = max(abs(dx), abs(dy), abs(dyaw))#only for one state change
        steps = max(int(dist / self.v_des), 1)

        x_step, y_step, yaw_step = dx / steps, dy / steps, dyaw / steps
        return [(x_start + i * x_step, y_start + i * y_step, yaw_start + i * yaw_step) for i in range(1, steps + 1)]

    def inverse_kinematics_batch(self, trajectory):
        sin = math.sin
        cos = math.cos
        sqrt = math.sqrt
        P_J1MC = np.array([[-250],[-318],[0]])
        P_J2MC = np.array([[250],[318],[0]])

        motor_trajectory_batch = []

        for x, y, yaw in trajectory:

            J1_x = x + cos(yaw)*P_J1MC[0,0] - sin(yaw)*P_J1MC[1,0]
            J1_y = y + sin(yaw)*P_J1MC[0,0] - cos(yaw)*P_J1MC[1,0]
            M1 = J1_x + sqrt(205.5**2-(abs(J1_y)-abs(P_J1MC[1,0]))**2)

            J2_x = x + cos(yaw)*P_J2MC[0,0] - sin(yaw)*P_J2MC[1,0]
            J2_y = y + sin(yaw)*P_J2MC[0,0] - cos(yaw)*P_J2MC[1,0]
            M2 = J2_x - sqrt(205.5**2-(abs(J2_y)-abs(P_J2MC[1,0]))**2)

            M3 = 0.0

            print("M1,M2",M1,M2)
            motor_trajectory_batch.append([M1,M2,M3])
        
        print("motor",motor_trajectory_batch)

        return motor_trajectory_batch

    def generate_home_trajectory(self, start, end):
        dM1_len, dM2_len, dM3_len = end[0] - start[0], end[1] - start[1], end[2] - start[2]
        max_dist = max(abs(dM1_len), abs(dM2_len), abs(dM3_len))
        steps = max(int(max_dist / self.v_des), 1)

        M1_step, M2_step, M3_step = dM1_len / steps, dM2_len / steps, dM3_len / steps
        return [[start[0] + i *M1_step, start[1] + i * M2_step, start[2] + i * M3_step] for i in range(1, steps + 1)]

    def pad_trajectory(self, trajectory):
        if len(trajectory) % self.batch_size != 0:
            last_point = trajectory[-1]
            needed = self.batch_size - (len(trajectory) % self.batch_size)
            trajectory += [last_point] * needed
        return trajectory

    def publish_motor_positions(self):
        if self.has_new_home and len(self.home_queue) > 0:
            batch = self.home_queue[:self.batch_size]
            self.home_queue = self.home_queue[self.batch_size:]
            motor_positions = batch  # already motor trajectory

            # queue_type = 'home'

        elif self.has_new_position and len(self.position_queue) > 0:
            batch = self.position_queue[:self.batch_size]
            self.position_queue = self.position_queue[self.batch_size:]
            motor_positions = self.inverse_kinematics_batch(batch)

            # queue_type = 'position'

        else:
            return  # Nothing to do
        
        # Save for target comparison
        self.last_sent_batch = motor_positions

        # Flatten and publish
        flat_positions = [val for pos in motor_positions for val in pos]
        msg = Float64MultiArray()
        msg.data = flat_positions
        self.motor_pub.publish(msg)
        self.get_logger().info(f"Published batch: {flat_positions}")

        # # Only check for completion AFTER publishing
        # if queue_type == 'home' and len(self.home_queue) == 0:
        #     self.has_new_home = False
        #     self.get_logger().info("Home trajectory complete.")
        #     self.check_motion = True
        # elif queue_type == 'position' and len(self.position_queue) == 0:
        #     self.has_new_position = False
        #     self.get_logger().info("Position trajectory complete.")
        #     self.check_motion = True

    def is_motor_at_target(self):
        if not self.last_sent_batch:
            return False
        last_target = np.array(self.last_sent_batch[-1])
        return np.allclose(self.current_motor_pos,last_target,atol=0.01)

    def timer_callback(self):
        #get set home and position
        if self.has_new_home or self.has_new_position:
            self.is_idle = False
            self.publish_motor_positions()
            self.motion_finished_pub.publish(Bool(data=False))     # publish trajectory is unfinished the FSM wouldn't publish

        # Only check for completion AFTER publishing
        if self.has_new_home and len(self.home_queue) == 0:
            self.get_logger().info("Home trajectory publish complete.")
            self.check_home_motion = True
        elif self.has_new_position  and len(self.position_queue) == 0:
            self.get_logger().info("Position trajectory publish complete.")
            self.check_position_motion = True

        #arrive home position?
        if self.check_home_motion:
            self.has_new_home = False
            if self.is_motor_at_target():
                self.motion_finished_pub.publish(Bool(data=True))  #home position is arrive
                self.init_finished_pub.publish(Bool(data=True))
                self.check_home_motion = False          
                self.is_idle =True
            else: 
                self.motion_finished_pub.publish(Bool(data=False))

        if self.check_position_motion:
            self.has_new_position = False
            if self.is_motor_at_target():
                self.motion_finished_pub.publish(Bool(data=True))  #home position is arrive
                self.check_position_motion = False          
                self.is_idle =True
            else: 
                self.motion_finished_pub.publish(Bool(data=False))

        if self.is_idle and not self.has_new_home and not self.has_new_position:
            # print("ready")
            self.motion_finished_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
