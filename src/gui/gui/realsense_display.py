import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from PySide6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer


class RealsenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.bridge = CvBridge()
        self.cv_image = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")


class RealsenseGUI(QWidget):
    def __init__(self, ros_node: RealsenseSubscriber):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Realsense Viewer")
        self.image_label = QLabel("等待影像中...")
        self.image_label.setFixedSize(1280, 720)

        self.save_button = QPushButton("📸 拍照儲存")
        self.save_button.clicked.connect(self.save_frame)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.save_button)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(30)  # 每 30ms 更新一次畫面 (~33 FPS)

    def update_image(self):
        cv_img = self.ros_node.cv_image
        if cv_img is not None:
            rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def save_frame(self):
        if self.ros_node.cv_image is not None:
            filename = "snapshot.png"
            cv2.imwrite(filename, self.ros_node.cv_image)
            print(f"✔ 影像已儲存至 {filename}")


def main():
    rclpy.init()
    ros_node = RealsenseSubscriber()

    app = QApplication(sys.argv)
    gui = RealsenseGUI(ros_node)
    gui.show()

    # 用 timer 呼叫 ROS spin，讓 PySide6 與 ROS2 能共存
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(10)

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
