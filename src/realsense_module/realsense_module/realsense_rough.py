import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32,Header
from sensor_msgs.msg import Image
import cv2
import os
import open3d as o3d
import numpy as np
from realsense_module import realsenselib as rslib

golden_path = "golden_sample.pcd"

class RealSenseRoughNode(Node):
    def __init__(self):
        super().__init__('realsense_rough_node')
        self.Cam = rslib.Cam_worker()
        self.golden_pcd = None

        # 啟動時嘗試讀取 golden sample
        if os.path.exists(golden_path):
            self.golden_pcd = o3d.io.read_point_cloud(golden_path)
            self.get_logger().info(f"📂 已載入 golden sample: {golden_path}")
        else:
            self.get_logger().warn("⚠️ 尚無 golden sample，請透過指令 'a' 拍攝建立")

        # publisher，發布彩色影像
        self.image_publisher= self.create_publisher(Image,'/camera/color/image_raw', 10)

        # 訂閱指令，支持拍攝新 golden sample ('a')、結束節點 ('q')等
        self.create_subscription(String, 'realsense_rough_cmd', self.cmd_callback, 10)

        # 發布比對距離
        self.dist_pub = self.create_publisher(Float32, 'realsense_rough_dist', 10)

        # 定時器，每 0.05 秒拍攝並比對（如果有 golden sample）
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.color_image = None
        self.depth_image = None
    

    def timer_callback(self):
        color_image, depth_image = self.Cam.take_pic()
        self.color_image = color_image
        self.depth_image = depth_image

        # Inside your timer callback or publish function
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_color_optical_frame'

        msg = Image(
            header=header,
            height=color_image.shape[0],
            width=color_image.shape[1],
            encoding='bgr8',
            is_bigendian=0,  # Must be int (0 = little endian)
            step=color_image.shape[1] * 3,
            data=color_image.tobytes()
        )

        self.image_publisher.publish(msg)

        
        if color_image is not None:
            # cv2.imshow('RealSense - Pose Detection', color_image)
            cv2.waitKey(1)

        if self.golden_pcd is not None:
            current_pcd = self.rs_to_pointcloud(color_image, depth_image, self.Cam.depth_intrin)
            if current_pcd is not None:
                dist = self.compare_pcd_distance(self.golden_pcd, current_pcd)
                self.get_logger().info(f"📏 平均點雲距離: {dist:.4f} m")
                msg = Float32()
                msg.data = float(dist)
                self.dist_pub.publish(msg)

    def cmd_callback(self, msg: String):
        cmd = msg.data.lower()
        if cmd == 'a':
            self.get_logger().info("📸 拍攝並儲存 golden sample")
            pcd = self.rs_to_pointcloud(self.color_image, self.depth_image, self.Cam.depth_intrin)
            if pcd is None:
                self.get_logger().warn("無影像資料，無法建立 golden sample")
                return
            o3d.io.write_point_cloud(golden_path, pcd)
            self.golden_pcd = pcd
            self.get_logger().info(f"✅ 成功儲存: {golden_path}")

        elif cmd == 'q':
            self.get_logger().info("🔚 收到結束指令，關閉節點")
            self.destroy_node()
            cv2.destroyAllWindows()

        else:
            self.get_logger().warn(f"❓ 未知指令: {cmd}。可用 'a' 建立 golden sample，'q' 結束節點")

    def rs_to_pointcloud(self, color, depth, intrin):
        if color is None or depth is None:
            return None
        h, w = depth.shape
        fx, fy = intrin.fx, intrin.fy
        cx, cy = intrin.ppx, intrin.ppy
        depth_scale = 0.001

        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth * depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        colors = color.reshape(-1, 3).astype(np.float32) / 255.0

        valid = (z.reshape(-1) > 0) & (z.reshape(-1) < 1.2)
        points = points[valid]
        colors = colors[valid]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def compare_pcd_distance(self, pcd1, pcd2):
        if pcd1 is None or pcd2 is None:
            return float('inf')
        dists = pcd1.compute_point_cloud_distance(pcd2)
        if len(dists) == 0:
            return float('inf')
        return np.mean(dists)

    def destroy_node(self):
        super().destroy_node()
        self.Cam.stop()
        self.get_logger().info("✅ 相機已停止")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRoughNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔚 程式終止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
