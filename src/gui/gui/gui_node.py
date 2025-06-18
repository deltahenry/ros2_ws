import sys
import cv2
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QGridLayout,
    QVBoxLayout, QHBoxLayout, QComboBox, QSizePolicy,QInputDialog
)
from PySide6.QtGui import QPixmap, QImage
from PySide6.QtCore import Qt, QTimer, QSize, Signal, QObject

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ButtonCmd, PoseIncrement, StateInfo, InterfaceMultipleMotors, InterfaceSingleMotor

from sensor_msgs.msg import Image
from std_msgs.msg import Bool,String,Float32
from cv_bridge import CvBridge
import copy
from uros_interface.srv import ESMcmd
from pymodbus.client import ModbusTcpClient

class RosNode_pub(Node):
    def __init__(self):
        super().__init__('my_ros_node_pub')  # 這樣可以確保多重繼承時的初始化順序正確
        self.get_logger().info('ROS 2 node initialized')

        # 建立一個發佈器，用於發佈名為 'test_int' 的 topic
        self.pose_pub = self.create_publisher(PoseIncrement, 'pose_increment', 10)
        self.button_cmd_publisher = self.create_publisher(ButtonCmd, '/button_cmd', 10)
        self.realsense_pub = self.create_publisher(String, '/realsense_rough_cmd', 10)
        # 初始化計數器
        self.counter = 0

    def publish_cmd(self, axis, step):
        # 發送 PoseIncrement
        pose_msg = PoseIncrement()
        pose_msg.axis = axis  # 0:x, 1:y, 2:yaw
        pose_msg.step = step  # -3, -2, -1, 1, 2, 3
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published PoseIncrement: {pose_msg}')

    def publish_button_cmd(self,button_cmd):
        button_cmd_msg = ButtonCmd()
        button_cmd_msg.init_button = button_cmd["init_cmd"]
        button_cmd_msg.battery_line_button = button_cmd["batter_line_cmd"]
        button_cmd_msg.cabinet_line_button = button_cmd["cabinet_line_cmd"]
        button_cmd_msg.manual_button = button_cmd["manual_cmd"]
        button_cmd_msg.gripper_button= button_cmd["gripper_cmd"]

        # print(button_cmd["init_cmd"])
        self.button_cmd_publisher.publish(button_cmd_msg)
class RosNode_sub(Node):
    def __init__(self):
        super().__init__('my_ros_node_sub')  # 這樣可以確保多重繼承時的初始化順序正確
        self.get_logger().info('ROS 2 node initialized')
        self.state_info_sub = self.create_subscription(StateInfo,'/state_info',self.state_info_callback,10)
        self.motion_finished_sub = self.create_subscription(Bool,'/motion_finished',self.motion_finished_callback,10)
        self.motors_info_sub = self.create_subscription(InterfaceMultipleMotors,'/multi_motor_info',self.motors_info_callback,10)
        self.rough_distance_sub = self.create_subscription(Float32,'/realsense_rough_dist',self.rough_distance_callback,10)
        self.motion_finished = False
        self.rough_distance = 1.2
        self.init_state_info()

    def init_state_info(self):
        self.state_info = StateInfo()
        self.state_info.initialize = False
        self.state_info.idle = False
        self.state_info.batterypicker = False
        self.state_info.batteryassembler = False
        self.state_info.error = False
        self.state_info.troubleshotting = False

    def state_info_callback(self, msg:StateInfo):
        self.state_info = msg
        # print("state",self.state_info.initialize)

    def motion_finished_callback(self,msg:Bool):
        self.motion_finished = msg.data  #False = the motor is moving
        # print("motion_finished",self.motion_finished)

    def motors_info_callback(self, msg:InterfaceMultipleMotors):
        self.current_motor_pos = [msg.motor_info[0].fb_position,msg.motor_info[1].fb_position,msg.motor_info[2].fb_position]
        # print("motor_info callback",self.current_motor_pos)

    def rough_distance_callback(self, msg:Float32):
        self.rough_distance = msg.data     # 更新粗略距離  
class RealsenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.get_logger().info('ROS 2 node initialized')
        self.bridge = CvBridge()
        self.cv_image = None  # ← Add this line
        self.latest_frame = None  # 儲存最新影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 根據實際情況調整
            self.image_callback,
            10
        )
        self.node_sub = RosNode_sub()

    def image_callback(self, msg):
        print("get image")
        try:
            # ROS Image → OpenCV Image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")   
        
class RosNode_client(Node):
    def __init__(self):
        super().__init__('ros_service_client_node')
        self.cli = self.create_client(ESMcmd, '/esm_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /esm_command service...')

    def call_servo_off(self):
        request = ESMcmd.Request()
        request.servo_status = False
        request.mode = 4
        request.speed_limit = 5
        request.lpf = 10

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().error('Service call failed')

    def call_servo_on(self):
        request = ESMcmd.Request()
        request.servo_status = True
        request.mode = 4
        request.speed_limit = 5
        request.lpf = 10

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().error('Service call failed')

class MyGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Layout Example")
        self.setGeometry(100, 100, 1200, 700)
        self.init_button_cmd()
        self.init_ui()
        self.step = 3
        self.update_count = 0
        self.y_value = None

        # 設備參數
        ip = "192.168.1.10"           # 請換成你的設備 IP
        port = 502                    # Modbus TCP 默認通訊埠
        self.slave_id = 2                   # 你的 Slave ID
        self.register_address = 0X9C60     # 要寫入的暫存器地址
        self.value_to_write = 0        # 寫入的值（16-bit 整數）

        # 建立連線
        self.client = ModbusTcpClient(ip, port=port)
        self.client.connect()

    def init_button_cmd(self):

        self.init_button =False
        self.battery_line_button=False
        self.cabinet_line_button =False
        self.manual_button =False
        self.gripper_button =False

    def init_ui(self):
        # === 圖片顯示區 ===
        self.image_label = QLabel()
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #2b2b2b; border: 1px solid gray;")

        # === 右上：狀態顯示區 ===
        self.top_right_text = QLabel("Status: Ready")  # 使用 QLabel 顯示靜態文字
        self.top_right_text.setAlignment(Qt.AlignCenter)
        self.top_right_text.setStyleSheet("background-color: #333; color: white; border: 1px solid gray;")

                # === Motion Buttons Grid ===
        button_names = ["X+", "X-", "Y+", "Y-", "Yaw+", "Yaw-"]
        button_grid = QGridLayout()
        self.right_buttons = []

        for i, name in enumerate(button_names):
            btn = QPushButton(name)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setMinimumSize(100, 60)
            btn.clicked.connect(getattr(self, f"on_{name.replace('+', 'plus').replace('-', 'minus').lower()}"))
            self.right_buttons.append(btn)
            button_grid.addWidget(btn, i // 2, i % 2)

        # === Quality Combo (above motion buttons, centered) ===
        self.quality_combo = QComboBox()
        self.quality_combo.addItems(["High", "Medium", "Low"])
        self.quality_combo.currentIndexChanged.connect(self.update_quality)
        self.quality_combo.setMaximumWidth(150)

        # Wrap quality combo and motion buttons in vertical layout
        quality_and_motion_layout = QVBoxLayout()
        quality_and_motion_layout.addWidget(self.quality_combo, alignment=Qt.AlignHCenter)
        quality_and_motion_layout.addSpacing(10)
        quality_and_motion_layout.addLayout(button_grid)

        # === Extra Buttons (right of motion buttons) ===
        extra_button_names = ["Assemble Place","Ready Place","Y-axis Home"]
        extra_button_callbacks = [self.on_extra1_clicked, self.on_extra2_clicked, self.on_extra3_clicked]
        extra_button_layout = QVBoxLayout()

        for name,callback in zip(extra_button_names,extra_button_callbacks):
            btn = QPushButton(name)
            btn.setMinimumWidth(40)
            btn.setMinimumHeight(40)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(callback)
            extra_button_layout.addWidget(btn)
        
        # === Combine motion block and extra block horizontally ===
        motion_and_extra_layout = QHBoxLayout()
        motion_and_extra_layout.addLayout(quality_and_motion_layout, 3)
        motion_and_extra_layout.addSpacing(10)
        motion_and_extra_layout.addLayout(extra_button_layout, 1)

        # === Final right panel: top text + combo + motion/extra ===
        right_top_layout = QVBoxLayout()
        right_top_layout.addWidget(self.top_right_text)
        right_top_layout.addSpacing(10)
        right_top_layout.addLayout(motion_and_extra_layout)

        # === Final Layout: image + right panel ===
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.image_label, 3)
        top_layout.addLayout(right_top_layout, 2)

        # === 下半部左：切換 1x6 或普通模式 ===
        self.layout_switcher = QComboBox()
        self.layout_switcher.addItems(["馬達位置、電流值", "IO設備狀態"])
        self.layout_switcher.currentIndexChanged.connect(self.switch_layout)

        self.stack_text = QWidget()
        self.stack_layout = QVBoxLayout()
        self.stack_text.setLayout(self.stack_layout)

        #for motor_info
        self.text_mode_1x6 = QWidget()
        row_layout = QHBoxLayout()
        self.motor_labels = []  # To store position/current QLabel references
        for i in range(6):
            motor_widget = QWidget()
            motor_layout = QVBoxLayout()

            motor_title = QLabel(f"<b>Motor {i + 1}</b>")
            position_label = QLabel("位置: ---")
            current_label = QLabel("電流 : ---")

            # Optional: set fixed width or alignment
            motor_title.setAlignment(Qt.AlignCenter)
            position_label.setAlignment(Qt.AlignLeft)
            current_label.setAlignment(Qt.AlignLeft)

            motor_layout.addWidget(motor_title)
            motor_layout.addWidget(position_label)
            motor_layout.addWidget(current_label)

            motor_widget.setLayout(motor_layout)
            row_layout.addWidget(motor_widget)

            self.motor_labels.append((position_label, current_label))

        self.text_mode_1x6.setLayout(row_layout)

        # 這裡修改為 QLabel 來顯示靜態文字，而不是 QTextEdit
        self.text_mode_plain = QLabel("""
            夾具: <b>On/Off</b><br>
            測距儀1: <b>數據</b><br>
            測距儀2: <b>數據</b>
        """)
        self.text_mode_plain.setAlignment(Qt.AlignCenter)
        # self.text_mode_plain.setStyleSheet("background-color: #2b2b2b; color: white; border: 1px solid gray;")

        self.stack_layout.addWidget(self.text_mode_1x6)
        self.stack_layout.addWidget(self.text_mode_plain)
        self.switch_layout(0)

        left_bottom_layout = QVBoxLayout()
        left_bottom_layout.addWidget(self.layout_switcher)
        left_bottom_layout.addWidget(self.stack_text)

        # === 下半部右：3x2 Button Layout ===9
        self.right_bottom_buttons = []
        button_grid_right = QGridLayout()

        # 按鈕名稱可以在這裡自定義
        right_button_names = ["初始化", "電池櫃輔助線",\
                              "夾具打開", "手動故障移除",\
                              "機櫃輔助線", "夾具關閉",\
                              "關閉馬達","ON馬達","拍攝Golden"]
        for i, button_name in enumerate(right_button_names):
            # 將中文按鈕名稱轉換為對應的函數名
            func_name = f"on_btn_{button_name.replace(' ', '').replace('初始化', 'initialize').replace('手動故障移除', 'manual_fault_reset').replace('電池櫃輔助線', 'battery_cabinet_aux').replace('機櫃輔助線', 'cabinet_aux').replace('夾具打開', 'gripper_open').replace('夾具關閉', 'gripper_close').replace('關閉馬達', 'servo_off').replace('ON馬達', 'servo_on').replace('拍攝Golden', 'take_golden')}"
            btn = QPushButton(f"{button_name}")
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.clicked.connect(getattr(self, func_name))

            # Save reference to the init button
            if button_name == "初始化":
                self.btn_initialize = btn  # <-- keep a reference
            elif button_name == "手動故障移除":
                self.btn_manual_fault_reset = btn  # <-- keep a reference
            elif button_name == "電池櫃輔助線":
                self.btn_battery_cabinet_aux = btn  # <-- keep a reference
            elif button_name == "機櫃輔助線":
                self.btn_cabinet_aux = btn  # <-- keep a reference
            elif button_name == "夾具打開":
                self.btn_gripper_open = btn  # <-- keep a reference
            elif button_name == "夾具關閉":
                self.btn_gripper_close = btn  # <-- keep a reference
            elif button_name == "關閉馬達":
                self.btn_servo_off = btn  # <-- keep a reference
            elif button_name == "ON馬達":
                self.btn_servo_on = btn  # <-- keep a reference
            elif button_name == "拍攝Golden":
                self.btn_take_golden = btn

            self.right_bottom_buttons.append(btn)
            button_grid_right.addWidget(btn, i // 3, i % 3)

        right_bottom_widget = QWidget()
        right_bottom_widget.setLayout(button_grid_right)

        # === 下半部 layout ===
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_bottom_layout, 3)
        bottom_layout.addWidget(right_bottom_widget, 2)

        # === 上下打包進主畫面 ===
        top_widget = QWidget()
        top_widget.setLayout(top_layout)
        top_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        bottom_widget = QWidget()
        bottom_widget.setLayout(bottom_layout)
        bottom_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        final_layout = QVBoxLayout()
        final_layout.addWidget(top_widget, 7)
        final_layout.addWidget(bottom_widget, 3)

        self.setLayout(final_layout)

        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self.update_frame)
        self.frame_timer.start(30)
        
        self.realsense_timer = QTimer(self)
        self.realsense_timer.timeout.connect(self.update_realsense_sub)
        self.realsense_timer.start(10)
        
        self.node_pub = RosNode_pub()
        self.node_sub = RosNode_sub()
        self.realsense_node = RealsenseSubscriber()
        self.ros_client = RosNode_client()

    def update_ros_sub(self):
        rclpy.spin_once(self.node_sub, timeout_sec=0.01)

    def update_realsense_sub(self):
        rclpy.spin_once(self.realsense_node, timeout_sec=0.01)

    def update_button_color(self):
       
        #init
        if self.init_button:
            self.btn_initialize.setStyleSheet("background-color: #3399FF; color: white;")        #blue
            # self.btn_initialize.setStyleSheet("background-color: #FFA500; color: white;") #orange
            if self.node_sub.state_info.initialize:
                if self.node_sub.motion_finished:  # motion_finished
                    self.btn_initialize.setStyleSheet("background-color: #4CAF50; color: white;") #green
                else:
                    self.btn_initialize.setStyleSheet("background-color: #FFA500; color: white;") #orange
        else:
            self.btn_initialize.setStyleSheet("background-color: #9E9E9E; color: black;") 
       
        #battery_picker
        if self.battery_line_button:
            self.btn_battery_cabinet_aux.setStyleSheet("background-color: #3399FF; color: white;")#blue
            # self.btn_battery_cabinet_aux.setStyleSheet("background-color: #FFA500; color: white;")
            if self.node_sub.state_info.batterypicker:
                if self.node_sub.motion_finished:  # motion_finished
                    self.btn_battery_cabinet_aux.setStyleSheet("background-color: #4CAF50; color: white;")
                else:
                    self.btn_battery_cabinet_aux.setStyleSheet("background-color: #FFA500; color: white;")
        else:
            self.btn_battery_cabinet_aux.setStyleSheet("background-color: #9E9E9E; color: black;")
       
        #battery_assembler
        if self.cabinet_line_button:
            self.btn_cabinet_aux.setStyleSheet("background-color: #3399FF; color: white;")#blue
            # self.btn_cabinet_aux.setStyleSheet("background-color: #FFA500; color: white;")
            if self.node_sub.state_info.batteryassembler:
                if self.node_sub.motion_finished:  # motion_finished
                    self.btn_cabinet_aux.setStyleSheet("background-color: #4CAF50; color: white;")
                else:
                    self.btn_cabinet_aux.setStyleSheet("background-color: #FFA500; color: white;")
        else:
            self.btn_cabinet_aux.setStyleSheet("background-color: #9E9E9E; color: black;")
        
        #trouble_shotting
        if self.manual_button:
            self.btn_manual_fault_reset.setStyleSheet("background-color: #3399FF; color: white;")#blue
            # self.btn_manual_fault_reset.setStyleSheet("background-color: #FFA500; color: white;")
            if self.node_sub.state_info.troubleshotting:
                if self.node_sub.motion_finished:  # motion_finished
                    self.btn_manual_fault_reset.setStyleSheet("background-color: #4CAF50; color: white;")
                else:
                    self.btn_manual_fault_reset.setStyleSheet("background-color: #FFA500; color: white;")
        else:
            self.btn_manual_fault_reset.setStyleSheet("background-color: #9E9E9E; color: black;")
       
        #gripper
        if self.gripper_button:
            self.btn_gripper_close.setStyleSheet("background-color: #4CAF50; color: white;")
            self.btn_gripper_open.setStyleSheet("background-color: #9E9E9E; color: black;")
        else:
            self.btn_gripper_close.setStyleSheet("background-color: #9E9E9E; color: black;")
            self.btn_gripper_open.setStyleSheet("background-color: #4CAF50; color: white;")
            
    def rewrite_button_cmd_state(self):
        self.init_button = copy.deepcopy(self.node_sub.state_info.initialize)
        self.battery_line_button = copy.deepcopy(self.node_sub.state_info.batterypicker)
        self.cabinet_line_button = copy.deepcopy(self.node_sub.state_info.batteryassembler)
        self.manual_button = copy.deepcopy(self.node_sub.state_info.troubleshotting)

    def update_motor_data(self):
        motor_data_list=[
            {'position': self.node_sub.current_motor_pos[0], 'current': 1.5},
            {'position': self.node_sub.current_motor_pos[1], 'current': 1.1},
            {'position': self.node_sub.current_motor_pos[2], 'current': 1.1},
            {'position': 0.0, 'current': 0.0},
            {'position': 0.0, 'current': 0.0},
            {'position': 0.0, 'current': 0.0},
        ]
        for i, data in enumerate(motor_data_list):
            if i < len(self.motor_labels):
                pos_label, curr_label = self.motor_labels[i]
                pos_label.setText(f"位置: {float(data['position']):.2f}")
                curr_label.setText(f"電流: {float(data['current']):.2f} A")

    #most important
    def update_frame(self):
        button_cmd = {
            "init_cmd":self.init_button,
            "batter_line_cmd":self.battery_line_button,
            "cabinet_line_cmd": self.cabinet_line_button,
            "manual_cmd":self.manual_button,
            "gripper_cmd":self.gripper_button
        }
        # print("init_cmd_before",self.init_button)
        self.node_pub.publish_button_cmd(button_cmd)
        
        self.update_ros_sub()
        self.update_motor_data()

        if self.update_count >=100:
            self.rewrite_button_cmd_state()
            self.update_count = 0
        

        # print("init_cmd_after",self.init_button)
        self.update_button_color()
        
        self.update_count +=1
        
        if self.realsense_node.cv_image is not None:
           # 加上紅色圓點（BGR = (0, 0, 255)）
            cv_image = self.realsense_node.cv_image
            h, w, _ = cv_image.shape
            points = [
            (50, 50),
            (w - 50, 50),
            (50, h - 50),
            (w - 50, h - 50)
            ]
            
            dist = self.node_sub.rough_distance
            threshold = 0.015  # 設定閾值 15mm
            point_color = (0, 0, 255)  # 預設紅色
            
            if dist < threshold:
                point_color = (0, 255, 0)  # 綠色

            # print("pick_in_image",self.node_sub.state_info.batterypicker)
            if  self.node_sub.state_info.batterypicker:
                for pt in points:
                    cv2.circle(cv_image, pt, radius=10, color=point_color, thickness=3)

                    cv2.line(cv_image, pt1=(295,364), pt2=(295,527), color=point_color, thickness=3)
                    cv2.line(cv_image, pt1=(1001,364), pt2=(1003,529), color=point_color, thickness=3)
                    cv2.line(cv_image, pt1=(1003,529), pt2=(295,527), color=point_color, thickness=3)

                    cv2.line(cv_image, pt1=(1018,514), pt2=(1018,714), color=point_color, thickness=3)
                    cv2.line(cv_image, pt1=(277,510), pt2=(277,714), color=point_color, thickness=3)

            elif self.node_sub.state_info.batteryassembler:
                for pt in points:
                    cv2.circle(cv_image, pt, radius=10, color=point_color, thickness=3)
                    cv2.line(cv_image, pt1=(50,50), pt2=(50,h-50), color=point_color, thickness=3)
                    cv2.line(cv_image, pt1=(w-50,50), pt2=(w-50,h-50), color=point_color, thickness=3)
            
            # OpenCV BGR → RGB
            RGB_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.latest_frame = RGB_cv_image

            frame = self.latest_frame
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.pixmap = QPixmap.fromImage(qimg)
        if hasattr(self, 'pixmap'):
            self.image_label.setPixmap(self.pixmap.scaled(
                self.image_label.width(),
                self.image_label.height(),
                Qt.IgnoreAspectRatio,
                Qt.SmoothTransformation
            ))
   
    # 每個按鈕的回調函數
    def on_xplus(self):
        self.top_right_text.setText("X+ Button clicked!")
        self.node_pub.publish_cmd(axis = 0, step = self.step)
   
    def on_xminus(self):
        self.top_right_text.setText("X- Button clicked!")
        self.node_pub.publish_cmd(axis = 0, step = self.step * (-1))
   
    def on_yplus(self):
        self.top_right_text.setText("Y+ Button clicked!")
        self.node_pub.publish_cmd(axis = 1, step = self.step)
   
    def on_yminus(self):
        self.top_right_text.setText("Y- Button clicked!")
        self.node_pub.publish_cmd(axis = 1, step = self.step * (-1))
   
    def on_yawplus(self):
        self.top_right_text.setText("Yaw+ Button clicked!")
        self.node_pub.publish_cmd(axis = 2, step = self.step)
   
    def on_yawminus(self):
        self.top_right_text.setText("Yaw- Button clicked!")
        self.node_pub.publish_cmd(axis = 2, step = self.step * (-1))
    
    def on_btn_initialize(self):
        self.top_right_text.setText("初始化 Button clicked!")

        if self.init_button:
            self.init_button = False
        else:
            self.init_button = True

    def on_btn_manual_fault_reset(self):
        self.top_right_text.setText("手動故障移除 Button clicked!")

        if self.manual_button:
            self.manual_button = False
        else:
            self.manual_button = True

    def on_btn_battery_cabinet_aux(self):
        self.top_right_text.setText("電池櫃輔助線 Button clicked!")

        if self.battery_line_button:
            self.battery_line_button = False
        else:
            self.battery_line_button = True

    def on_btn_cabinet_aux(self):
        self.top_right_text.setText("機櫃輔助線 Button clicked!")

        if self.cabinet_line_button:
            self.cabinet_line_button = False
        else:
            self.cabinet_line_button = True

    def on_btn_gripper_open(self):
        self.top_right_text.setText("夾具打開 Button clicked!")
        self.client.write_register(address=self.register_address, value=49152, slave=self.slave_id)
        self.gripper_button =False
        
    def on_btn_gripper_close(self):
        self.top_right_text.setText("夾具關閉 Button clicked!")
        self.client.write_register(address=self.register_address, value=0, slave=self.slave_id)
        self.gripper_button =True
        
    def on_quality_high(self):
        self.top_right_text.setText("High quality selected!")
        self.step = 3

    def on_quality_medium(self):
        self.top_right_text.setText("Medium quality selected!")
        self.step = 2
    
    def on_quality_low(self):
        self.top_right_text.setText("Low quality selected!")
        self.step = 1
        
    def update_quality(self, index):
        if index == 0:
            self.on_quality_high()
        elif index == 1:
            self.on_quality_medium()
        elif index == 2:
            self.on_quality_low()

    def on_btn_servo_off(self):
        self.top_right_text.setText("Sending Servo OFF command...")
        self.ros_client.call_servo_off()
        self.top_right_text.setText("Servo OFF command sent.")

    def on_btn_servo_on(self):
        self.top_right_text.setText("Sending Servo ON command...")
        self.ros_client.call_servo_on()
        self.top_right_text.setText("Servo ON command sent.")

    def on_extra1_clicked(self):
        self.top_right_text.setText("Assemble Place clicked")
        self.node_pub.publish_cmd(axis = 3, step = 4)

    def on_extra2_clicked(self):
        self.top_right_text.setText("Ready Place clicked")
        self.node_pub.publish_cmd(axis = 4, step = 4)

    def on_extra3_clicked(self):
        self.top_right_text.setText("Y-axis Home Place clicked")
        self.node_pub.publish_cmd(axis = 5, step = 4)

    def on_btn_take_golden(self):
        print("🟡 發送拍攝 golden sample 指令中...")
        msg = String()
        msg.data = "a"
        self.node_pub.realsense_pub.publish(msg)
        print("✅ 已送出拍攝指令：take_golden")

    def switch_layout(self, index):
        self.text_mode_1x6.setVisible(index == 0)
        self.text_mode_plain.setVisible(index == 1)

    def load_static_image(self, path):
        self.pixmap = QPixmap(path)
        if self.pixmap.isNull():
            print(f"Warning: Failed to load image: {path}")
            return
        self.update_image_display()
    
    def update_image_display(self):
        if hasattr(self, "pixmap") and self.pixmap:
            scaled = self.pixmap.scaled(
                self.image_label.size(),
                Qt.IgnoreAspectRatio,  # 不保持比例，完全填滿區域
                Qt.SmoothTransformation
            )
            self.image_label.setPixmap(scaled)
    
    def resizeEvent(self, event):
        self.update_image_display()
        super().resizeEvent(event)

    def closeEvent(self, event):
        event.accept()

def main():
    rclpy.init()  # 初始化 ROS 2

    app = QApplication(sys.argv)
    window = MyGUI()
    window.show()

    sys.exit(app.exec())
    rclpy.shutdown()  # 關閉 ROS 2
if __name__ == "_main_":
    main()