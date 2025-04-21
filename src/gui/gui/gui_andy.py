import sys
import cv2
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QGridLayout,
    QVBoxLayout, QHBoxLayout, QComboBox, QSizePolicy
)
from PySide6.QtGui import QPixmap, QImage
from PySide6.QtCore import Qt, QTimer, QSize, Signal, QObject

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ButtonCmd, PoseIncrement, StateInfo

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosNode_pub(Node):
    def __init__(self):
        super().__init__('my_ros_node_pub')  # 這樣可以確保多重繼承時的初始化順序正確
        self.get_logger().info('ROS 2 node initialized')

        # 建立一個發佈器，用於發佈名為 'test_int' 的 topic
        self.pose_pub = self.create_publisher(PoseIncrement, 'pose_increment', 10)
        # 初始化計數器
        self.counter = 0

    def publish_cmd(self, axis, step):

        # 發送 PoseIncrement
        pose_msg = PoseIncrement()
        pose_msg.axis = axis  # 0:x, 1:y, 2:yaw
        pose_msg.step = step  # -3, -2, -1, 1, 2, 3
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published PoseIncrement: {pose_msg}')

class RealsenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.get_logger().info('ROS 2 node initialized')
        self.bridge = CvBridge()
        self.latest_frame = None  # 儲存最新影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 根據實際情況調整
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # print("In call back")
            # ROS Image → OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 加上紅色圓點（BGR = (0, 0, 255)）
            h, w, _ = cv_image.shape
            points = [
            (50, 50),
            (w - 50, 50),
            (50, h - 50),
            (w - 50, h - 50)
            ]
            for pt in points:
                cv2.circle(cv_image, pt, radius=10, color=(0, 0, 255), thickness=3)
            # OpenCV BGR → RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.latest_frame = cv_image
            # print("Received img")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")   
        
class MyGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Layout Example")
        self.setGeometry(100, 100, 1200, 700)

        self.init_ui()
        self.load_static_image("cabinet.jpg")  # 載入圖片而非 OpenCV 攝影機
        self.step = 3
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

        self.right_buttons = []
        button_grid = QGridLayout()

        # 按鈕名稱可以在這裡自定義
        button_names = ["X+", "X-", "Y+", "Y-", "Yaw+", "Yaw-"]
        for i, button_name in enumerate(button_names):
            btn = QPushButton(button_name)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.clicked.connect(getattr(self, f"on_{button_name.replace('+', 'plus').replace('-', 'minus').lower()}"))
            self.right_buttons.append(btn)
            button_grid.addWidget(btn, i // 2, i % 2)

        self.quality_combo = QComboBox()
        self.quality_combo.addItems(["High", "Medium", "Low"])
        self.quality_combo.currentIndexChanged.connect(self.update_quality)

        button_and_combo_layout = QHBoxLayout()
        button_and_combo_layout.addLayout(button_grid, 3)
        button_and_combo_layout.addWidget(self.quality_combo, 1)

        right_top_layout = QVBoxLayout()
        right_top_layout.addWidget(self.top_right_text)
        right_top_layout.addLayout(button_and_combo_layout)

        # === 上半部 layout：左影像 + 右操作 ===
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

        self.text_mode_1x6 = QWidget()
        grid = QGridLayout()
        for i in range(6):
            label = QLabel(f"Motor {i + 1}")
            label.setAlignment(Qt.AlignCenter)
            grid.addWidget(label, 0, i)
        self.text_mode_1x6.setLayout(grid)

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

        # === 下半部右：3x2 Button Layout ===
        self.right_bottom_buttons = []
        button_grid_right = QGridLayout()

        # 按鈕名稱可以在這裡自定義
        right_button_names = ["初始化", "電池櫃輔助線",\
                              "夾具打開", "手動故障移除",\
                              "機櫃輔助線", "夾具關閉"]
        for i, button_name in enumerate(right_button_names):
            # 將中文按鈕名稱轉換為對應的函數名
            func_name = f"on_btn_{button_name.replace(' ', '').replace('初始化', 'initialize').replace('手動故障移除', 'manual_fault_reset').replace('電池櫃輔助線', 'battery_cabinet_aux').replace('機櫃輔助線', 'cabinet_aux').replace('夾具打開', 'gripper_open').replace('夾具關閉', 'gripper_close')}"
            btn = QPushButton(f"{button_name}")
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.clicked.connect(getattr(self, func_name))
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
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)
        
        self.node_pub = RosNode_pub()
        self.realsense_node = RealsenseSubscriber()
    def update_ros_sub(self):
        rclpy.spin_once(self.realsense_node, timeout_sec=0.01)
    def update_frame(self):
        if self.realsense_node.latest_frame is not None:
            frame = self.realsense_node.latest_frame
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

    def on_btn_manual_fault_reset(self):
        self.top_right_text.setText("手動故障移除 Button clicked!")

    def on_btn_battery_cabinet_aux(self):
        self.top_right_text.setText("電池櫃輔助線 Button clicked!")

    def on_btn_cabinet_aux(self):
        self.top_right_text.setText("機櫃輔助線 Button clicked!")

    def on_btn_gripper_open(self):
        self.top_right_text.setText("夾具打開 Button clicked!")

    def on_btn_gripper_close(self):
        self.top_right_text.setText("夾具關閉 Button clicked!")
        
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