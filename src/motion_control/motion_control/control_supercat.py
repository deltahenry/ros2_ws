import numpy as np
import math
import copy

class RobotController:
    def __init__(self):

        # 机器人参数
        self.P_J1MC = np.array([[-250], [-318], [0]])  # 关节1在机器人坐标系中的位置
        self.P_J2MC = np.array([[250], [318], [0]])    # 关节2在机器人坐标系中的位置
        
        # 设备状态
        self.current_pose = [0, 0, 0]                  # 当前设备位置 [x, y, yaw]
        self.current_motor_positions = [0, 0, 0]       # 当前马达位置 [M1, M2, M3]
        
        # 轨迹规划参数
        self.trajectory = []                           # 位姿轨迹
        self.motor_trajectory = []                     # 马达轨迹
        self.current_trajectory_index = 0              # 当前轨迹点索引
        
        # 马达参数
        self.motor_reached_threshold = 0.5             # 马达到达目标位置的阈值
    
    def set_initial_state(self, initial_pose, initial_motor_positions):

        self.current_pose = copy.deepcopy(initial_pose)
        self.current_motor_positions = copy.deepcopy(initial_motor_positions)
        print(f"initial device pose: {self.current_pose}")
        print(f"initial motor position: {self.current_motor_positions}")
    
    def generate_trajectory(self, start_pose, end_pose,v_des):

        x_start, y_start, yaw_start = start_pose[0],start_pose[1],start_pose[2]
        x_end, y_end, yaw_end = end_pose[0],end_pose[1],end_pose[2]

        dx, dy, dyaw = x_end - x_start, y_end - y_start, yaw_end - yaw_start
        dist = max(abs(dx), abs(dy), abs(dyaw))
        
        steps = max(int(dist /v_des), 1)
        
        # 计算每一步的增量
        x_step, y_step, yaw_step = dx / steps, dy / steps, dyaw / steps
        
        # 生成轨迹点
        trajectory = []
        for i in range(steps + 1):  # 包括起点和终点
            x = x_start + i * x_step
            y = y_start + i * y_step
            yaw = yaw_start + i * yaw_step
            trajectory.append((x, y, yaw))
        
        return trajectory
    
    def inverse_kinematics(self, trajectory,initial_motor_positions):
        
        sin = math.sin
        cos = math.cos
        sqrt = math.sqrt
        motor_trajectory = []

        for pose in trajectory:
            x, y, yaw = pose

            J1_x = x + cos(yaw) * self.P_J1MC[0, 0] - sin(yaw) * self.P_J1MC[1, 0]
            J1_y = sin(yaw) * self.P_J1MC[0, 0] - cos(yaw) * self.P_J1MC[1, 0]

            J2_x = x + cos(yaw) * self.P_J2MC[0, 0] - sin(yaw) * self.P_J2MC[1, 0]
            J2_y = sin(yaw) * self.P_J2MC[0, 0] - cos(yaw) * self.P_J2MC[1, 0]
            
            M1 = J1_x + sqrt(205.5**2 - (abs(J1_y) - abs(self.P_J1MC[1, 0]))**2) - initial_motor_positions[0]
            M2 = J2_x - sqrt(205.5**2 - (abs(J2_y) - abs(self.P_J2MC[1, 0]))**2) - initial_motor_positions[1]
            M3 = y - initial_motor_positions[2]

            motor_trajectory.append([M1, M2, M3])
        
        return motor_trajectory
      
    def reach_target(self,current_motor_positions,motor_trajectory):

        for i in range(3):
            if abs(motor_trajectory[i] - current_motor_positions[i]) > self.motor_reached_threshold:
                return False
        return True
    
    def update_current_pose(self,current_motor_positions,target_motor_positions,end_pose):
        
        reach = self.reach_target(current_motor_positions,target_motor_positions)

        if reach:
            self.current_pose = end_pose
    
        return self.current_pose


# 使用示例
def main():
    # 创建控制器
    controller = RobotController()  # 较大的速度值意味着较少的轨迹点
    
    # 设置初始状态
    initial_pose = [209.0, 0.0, 0.0]          #[x, y, yaw]
    initial_motor_positions = [164.5, 253.5, 0.0]
    controller.set_initial_state(initial_pose, initial_motor_positions)
    
    # 设置目标位姿，生成轨迹, base on desired velocity
    start_pose =  [209.0, 0.0, 0.0]
    end_pose = [481.0, 0.0, 0.0]
    v_des = 1

    #generate motor trajectory
    trajectory = controller.generate_trajectory(start_pose, end_pose,v_des)
    motor_trajectory = controller.inverse_kinematics(trajectory,initial_motor_positions)
    print(f"\n马达轨迹: {motor_trajectory}")


    target_motor_positions = motor_trajectory[-1]  #last set
    current_motor_positions = target_motor_positions  #need to monitor real motor position
    # current_motor_positions = [0.0,0.0,0.0]

    #update current pose
    current_pose = controller.update_current_pose(current_motor_positions,target_motor_positions,end_pose)
    print("current_pose",current_pose)

if __name__ == "_main_":
    main()