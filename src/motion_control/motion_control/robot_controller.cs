#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <algorithm> // for std::max

using namespace std;
using namespace Eigen;

class RobotController {
public:
    // 机器人参数
    MatrixXd P_J1MC;
    MatrixXd P_J2MC;
    
    // 设备状态
    vector<double> current_pose;                  // 当前设备位置 [x, y, yaw]
    vector<double> current_motor_positions;       // 当前马达位置 [M1, M2, M3]
    
    // 轨迹规划参数
    vector< vector<double> > trajectory;          // 位姿轨迹
    vector< vector<double> > motor_trajectory;      // 马达轨迹
    int current_trajectory_index;                 // 当前轨迹点索引
    
    // 马达参数
    double motor_reached_threshold;               // 马达到达目标位置的阈值

    RobotController() {
        // Initialize robot parameters
        P_J1MC = MatrixXd(3, 1);
        P_J1MC(0, 0) = -250;
        P_J1MC(1, 0) = -318;
        P_J1MC(2, 0) = 0;
        
        P_J2MC = MatrixXd(3, 1);
        P_J2MC(0, 0) = 250;
        P_J2MC(1, 0) = 318;
        P_J2MC(2, 0) = 0;
        
        // Initialize device state
        current_pose = {0, 0, 0};                   // 当前设备位置 [x, y, yaw]
        current_motor_positions = {0, 0, 0};          // 当前马达位置 [M1, M2, M3]
        
        // Initialize trajectory planning parameters
        trajectory = {};                            // 位姿轨迹
        motor_trajectory = {};                      // 马达轨迹
        current_trajectory_index = 0;               // 当前轨迹点索引
        
        // Initialize motor parameters
        motor_reached_threshold = 0.5;              // 马达到达目标位置的阈值
    }
    
    void set_initial_state(const vector<double>& initial_pose, const vector<double>& initial_motor_positions) {
        // 深拷贝初始状态
        this->current_pose = initial_pose;
        this->current_motor_positions = initial_motor_positions;
        cout << "initial device pose: ";
        for (size_t i = 0; i < this->current_pose.size(); i++) {
            cout << this->current_pose[i] << " ";
        }
        cout << endl;
        cout << "initial motor position: ";
        for (size_t i = 0; i < this->current_motor_positions.size(); i++) {
            cout << this->current_motor_positions[i] << " ";
        }
        cout << endl;
    }
    
    vector< vector<double> > generate_trajectory(const vector<double>& start_pose, const vector<double>& end_pose, double v_des) {
        double x_start = start_pose[0];
        double y_start = start_pose[1];
        double yaw_start = start_pose[2];
        double x_end = end_pose[0];
        double y_end = end_pose[1];
        double yaw_end = end_pose[2];
        
        double dx = x_end - x_start;
        double dy = y_end - y_start;
        double dyaw = yaw_end - yaw_start;
        double dist = max({fabs(dx), fabs(dy), fabs(dyaw)});
        
        int steps = max(static_cast<int>(dist / v_des), 1);
        
        // 计算每一步的增量
        double x_step = dx / steps;
        double y_step = dy / steps;
        double yaw_step = dyaw / steps;
        
        // 生成轨迹点
        vector< vector<double> > trajectory;
        for (int i = 0; i <= steps; i++) {  // 包括起点和终点
            double x = x_start + i * x_step;
            double y = y_start + i * y_step;
            double yaw = yaw_start + i * yaw_step;
            trajectory.push_back({x, y, yaw});
        }
        
        return trajectory;
    }
    
    vector< vector<double> > inverse_kinematics(const vector< vector<double> >& trajectory, const vector<double>& initial_motor_positions) {
        // 使用 math 库函数
        // sin, cos, sqrt 分别等价于 std::sin, std::cos, std::sqrt
        vector< vector<double> > motor_trajectory;
        
        for (size_t i = 0; i < trajectory.size(); i++) {
            double x = trajectory[i][0];
            double y = trajectory[i][1];
            double yaw = trajectory[i][2];
            
            double J1_x = x + cos(yaw) * P_J1MC(0, 0) - sin(yaw) * P_J1MC(1, 0);
            double J1_y = sin(yaw) * P_J1MC(0, 0) - cos(yaw) * P_J1MC(1, 0);
            
            double J2_x = x + cos(yaw) * P_J2MC(0, 0) - sin(yaw) * P_J2MC(1, 0);
            double J2_y = sin(yaw) * P_J2MC(0, 0) - cos(yaw) * P_J2MC(1, 0);
            
            double term1 = fabs(J1_y) - fabs(P_J1MC(1, 0));
            double term2 = fabs(J2_y) - fabs(P_J2MC(1, 0));
            double M1 = J1_x + sqrt((205.5 * 205.5) - (term1 * term1)) - initial_motor_positions[0];
            double M2 = J2_x - sqrt((205.5 * 205.5) - (term2 * term2)) - initial_motor_positions[1];
            double M3 = y - initial_motor_positions[2];
            
            motor_trajectory.push_back({M1, M2, M3});
        }
        
        return motor_trajectory;
    }
      
    bool reach_target(const vector<double>& current_motor_positions, const vector<double>& motor_trajectory) {
        for (int i = 0; i < 3; i++) {
            if (fabs(motor_trajectory[i] - current_motor_positions[i]) > motor_reached_threshold) {
                return false;
            }
        }
        return true;
    }
    
    vector<double> update_current_pose(const vector<double>& current_motor_positions, const vector<double>& target_motor_positions, const vector<double>& end_pose) {
        bool reach = reach_target(current_motor_positions, target_motor_positions);
        
        if (reach) {
            current_pose = end_pose;
        }
        
        return current_pose;
    }
};

// 使用示例
void main_function() {
    // 创建控制器
    RobotController controller;  // 较大的速度值意味着较少的轨迹点
    
    // 设置初始状态
    vector<double> initial_pose = {209.0, 0.0, 0.0};          // [x, y, yaw]
    vector<double> initial_motor_positions = {164.5, 253.5, 0.0};
    controller.set_initial_state(initial_pose, initial_motor_positions);
    
    // 设置目标位姿，生成轨迹, base on desired velocity
    vector<double> start_pose = {209.0, 0.0, 0.0};
    vector<double> end_pose = {481.0, 0.0, 0.0};
    double v_des = 1;
    
    //generate motor trajectory
    vector< vector<double> > trajectory = controller.generate_trajectory(start_pose, end_pose, v_des);
    vector< vector<double> > motor_trajectory = controller.inverse_kinematics(trajectory, initial_motor_positions);
    cout << "\n马达轨迹: ";
    for (size_t i = 0; i < motor_trajectory.size(); i++) {
        cout << "[";
        for (size_t j = 0; j < motor_trajectory[i].size(); j++) {
            cout << motor_trajectory[i][j];
            if (j < motor_trajectory[i].size() - 1) {
                cout << ", ";
            }
        }
        cout << "] ";
    }
    cout << endl;
    
    vector<double> target_motor_positions = motor_trajectory[motor_trajectory.size() - 1];  // last set
    vector<double> current_motor_positions = target_motor_positions;  // need to monitor real motor position
    // current_motor_positions = {0.0,0.0,0.0};
    
    //update current pose
    vector<double> current_pose = controller.update_current_pose(current_motor_positions, target_motor_positions, end_pose);
    cout << "current_pose: ";
    for (size_t i = 0; i < current_pose.size(); i++) {
        cout << current_pose[i] << " ";
    }
    cout << endl;
}

int main() {
    // In Python, the condition is if __name__ == "_main_"
    // Here we call main_function() directly.
    main_function();
    return 0;
}
