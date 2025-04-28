from uros_interface.msg import Joint2DArr,JointArr
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import rclpy
import numpy as np

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Publishers
        self.esp_control_publisher = self.create_publisher(Joint2DArr,'/theta_command',10)

        #Subscribers
        self.pos_ref_sub = self.create_subscription(Float32MultiArray, '/motor_position_ref',self.pos_ref_callback , 10) #motor_node
        self.esp_info_subscriber = self.create_subscription(JointArr, '/theta_feedback', self.esp_info_callback ,10)
    
        # Timer
        self.timer = self.create_timer(2, self.timer_callback)  #20Hz

        self.motors_len = []
       

    def pos_ref_callback(self,msg:Float32MultiArray):
        pos_ref_queue = np.reshape(msg.data,(10,3))
        self.pub_esp_cmd(pos_ref_queue)

    def esp_info_callback(self,msg:JointArr):
        self.motors_len = msg.theta_arr
        # print(self.motors_len)
    
    def pub_esp_cmd(self,pos_ref_queue):
        msg = Joint2DArr()
        msg.theta_2d_arr = [JointArr() for _ in range(10)]
        for i in range (10):
            # msg.theta_2d_arr[i].theta_arr = [5.0,5.0,5.0,0.0,0.0,0.0]
            print([pos_ref_queue[i][0],pos_ref_queue[i][1],pos_ref_queue[i][2],0.0,0.0,0.0])
            msg.theta_2d_arr[i].theta_arr = [float(pos_ref_queue[i][0]),float(pos_ref_queue[i][1]),float(pos_ref_queue[i][2]),0.0,0.0,0.0]
        self.esp_control_publisher.publish(msg)

    def timer_callback(self):
        """This runs at 20Hz."""
        print("motor_control")

def main(args=None):
    rclpy.init(args=args)
    ui_node = UINode()
    rclpy.spin(ui_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()