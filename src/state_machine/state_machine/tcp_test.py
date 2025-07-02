#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 你可以改成你要的 message type
from custom_msgs.msg import PoseIncrement
import math


class ExampleSubscriber(Node):
    def __init__(self):
        super().__init__('example_subscriber')
        self.pose_increment_sub = self.create_subscription(PoseIncrement,'pose_increment',self.pose_increment_callback,10)
        self.init_pose()

    def init_pose(self):
        # Initialize the pose variables
        self.x_move = 0.0
        self.y_move = 0.0
        self.yaw_move = 0.0
        self.delta_y = 1180.0

        #center pose
        self.Xc_x = 345.0
        self.clipper_len = 0.0
        self.Xc_yaw = 0.0

    def tcp_transform(self, x_move, yaw_move, delta_y):

        Xp_x = x_move*math.cos(yaw_move)
        Xp_y = -x_move*math.sin(yaw_move) + delta_y

        Xcc_x = delta_y*math.sin(yaw_move) + x_move*math.cos(yaw_move)
        Xcc_y = -delta_y*math.cos(yaw_move) + self.x_move*math.sin(yaw_move) + delta_y

        m = (Xcc_y - Xp_y) / (Xcc_x - Xp_x) if (Xcc_x - Xp_x) != 0 else float('inf')  # slope

        Xc_y = 0
        Xc_x = (Xc_y - Xcc_y)/ m + Xcc_x if m != 0 else Xcc_x  # x coordinate of the center point
        clipper_len = math.sqrt((Xc_x-Xcc_x)**2 + (Xc_y-Xcc_y)**2)  # distance from the center point to the clipper

        Xc_x = Xc_x + 345.0  # offset for the center point

        return Xc_x, clipper_len, yaw_move


    def pose_increment_callback(self, msg=PoseIncrement):
        base_increments = {
            0: 1.0,        # x
            1: 1.0,        # y
            # 2: 0.000436       # yaw (0.025 degree in radians)
            2: 0.01745       # yaw (1 degree in radians)
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
            self.x_move += delta
            self.Xc_x,self.clipper_len,self.Xc_yaw = self.tcp_transform(self.x_move, self.yaw_move, self.delta_y)
        elif msg.axis == 1:
            self.y_move += delta
            self.clipper_len += delta
        elif msg.axis == 2:
            self.yaw_move += delta
            self.Xc_x,self.clipper_len,self.Xc_yaw = self.tcp_transform(self.x_move, self.yaw_move, self.delta_y)

        elif msg.axis == 3: #assemble place
            self.clipper_len = 1200.0

        elif msg.axis == 4: #ready place
            self.clipper_len = 450.0
            
        elif msg.axis == 5: #y-axis home place
            self.clipper_len = 0.0
        else:
            print("Invalid axis")

        print("Current pose:")
        print("Xc_x:", self.Xc_x)
        print("clipper_len:", self.clipper_len)
        print("Xc_yaw:", self.Xc_yaw)
        


        #safety range:
        x1 = -8.26*self.Xc_yaw + 475.0
        x2 = 8.26*self.Xc_yaw + 475.0
        x3 = -8.26*self.Xc_yaw + 215.0
        x4 = 8.26*self.Xc_yaw + 215.0

        #pos cmd safety check
        if self.clipper_len > -20.0 and self.clipper_len < 1250:
            if self.Xc_yaw >=0:
                if self.Xc_x <= x1 and self.Xc_x >= x4 and self.Xc_yaw <= 0.087:
                    print("pos cmd safety check passed")
                    # self.blackboard["pos_cmd"]["x"] = self.Xc_x
                    # self.blackboard["pos_cmd"]["y"] = self.clipper_len
                    # self.blackboard["pos_cmd"]["yaw"] = self.Xc_yaw
                else:
                    print("pos cmd safety check failed")
            elif self.Xc_yaw <0:
                if self.Xc_x <= x2 and self.Xc_x >= x3 and self.Xc_yaw >= -0.087:
                    print("pos cmd safety check passed")
                    # self.blackboard["pos_cmd"]["x"] = self.Xc_x
                    # self.blackboard["pos_cmd"]["y"] = self.clipper_len
                    # self.blackboard["pos_cmd"]["yaw"] = self.Xc_yaw
                else:
                    print("pos cmd safety check failed")
        else:
            print("exceed safety range")


def main(args=None):
    rclpy.init(args=args)
    node = ExampleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
