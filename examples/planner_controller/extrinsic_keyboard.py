# ...existing code...
"""
camera_pcd_example.py - Script to check the point cloud extrinsics and publish to RViz2

"""
from dexe_robot_client.camera_client import OrbbecClient
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header

# ====== 新增：非阻塞键盘监听支持（Linux 终端） ======
import sys
import tty
import termios
import select

def numpy_to_pointcloud2(points: np.ndarray, frame_id="base_link") -> PointCloud2:
    """
    numpy points: shape (N, 3), dtype float32
    """
    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = frame_id

    return point_cloud2.create_cloud_xyz32(header, points.tolist())

# 键盘监听线程
class KeyboardListener(threading.Thread):
    def __init__(self, angles_ref, trans_ref, lock, rot_step=0.01, trans_step=0.005):
        super().__init__(daemon=True)
        self.angles_ref = angles_ref  # list reference
        self.trans_ref = trans_ref    # list reference
        self.lock = lock
        self.rot_step = rot_step
        self.trans_step = trans_step
        self.running = True
        self._old_settings = None

    def run(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            print("[键盘控制] 方向键: XY 平移 | PgUp/PgDn: Z 平移 | 1/2: Z 角 | 3/4: Y 角 | 5/6: Z2 角 | r: 重置 | q: 退出")
            while self.running:
                if self._key_available():
                    key = self._read_key()
                    self._handle_key(key)
                else:
                    time.sleep(0.01)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def stop(self):
        self.running = False

    def _key_available(self):
        return select.select([sys.stdin], [], [], 0)[0]

    def _read_key(self):
        ch1 = sys.stdin.read(1)
        # 处理方向键 / 功能键序列
        if ch1 == '\x1b':  # ESC 开头的序列
            if select.select([sys.stdin], [], [], 0.005)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    if ch3 in ('A','B','C','D'):  # 箭头
                        return f'ARROW_{ch3}'
                    # PageUp/PageDown 等可能是两个字符，如 5~ / 6~
                    if ch3 in ('5','6'):
                        ch4 = sys.stdin.read(1)  # 应为 ~
                        if ch3 == '5':
                            return 'PAGE_UP'
                        elif ch3 == '6':
                            return 'PAGE_DOWN'
        return ch1

    def _handle_key(self, key):
        with self.lock:
            # 平移控制
            if key in ('w', 'W'):      # Up -> +Y
                self.trans_ref[1] += self.trans_step
            elif key in ('s', 'S'):    # Down -> -Y
                self.trans_ref[1] -= self.trans_step
            elif key in ('d', 'D'):    # Right -> +X
                self.trans_ref[0] += self.trans_step
            elif key in ('a', 'A'):    # Left -> -X
                self.trans_ref[0] -= self.trans_step
            elif key in ('e', 'E'):    # +Z
                self.trans_ref[2] += self.trans_step
            elif key in ('q', 'Q'):  # -Z
                self.trans_ref[2] -= self.trans_step

            # 旋转控制：zyz 欧拉角
            elif key == '1':  # Z +
                self.angles_ref[0] += self.rot_step
            elif key == '2':  # Z -
                self.angles_ref[0] -= self.rot_step
            elif key == '3':  # Y +
                self.angles_ref[1] += self.rot_step
            elif key == '4':  # Y -
                self.angles_ref[1] -= self.rot_step
            elif key == '5':  # Z2 +
                self.angles_ref[2] += self.rot_step
            elif key == '6':  # Z2 -
                self.angles_ref[2] -= self.rot_step

            elif key in ('r', 'R'):
                # 重置为初始值
                self.angles_ref[0] = -0.1
                self.angles_ref[1] = np.pi/2 - 0.1
                self.angles_ref[2] = -0.05 - 0.1
                self.trans_ref[0] = 0.01 - 0.01
                self.trans_ref[1] = -0.1605
                self.trans_ref[2] = 0.02 - 0.01

            elif key in ('q', 'Q'):
                self.running = False
                print("退出中...")

            # 打印当前值
            print(f"[外参] angles(zyz)={self.angles_ref}, translation={self.trans_ref}")

if __name__ == "__main__":
    if not rclpy.ok():
        rclpy.init()
    orbbec_client = OrbbecClient()
    pcd_executor = MultiThreadedExecutor()
    pcd_executor.add_node(orbbec_client)
    
    pub_pcd_node = Node("pub_pcd_node")
    pcd_env_publisher = pub_pcd_node.create_publisher(
        PointCloud2, "/dexe_robot_client/rviz2/point_cloud", 1
    )
    pcd_executor.add_node(pub_pcd_node)

    ros_thread = threading.Thread(target=pcd_executor.spin, daemon=True)
    ros_thread.start()

    while True:
        if not orbbec_client.is_connected():
            time.sleep(0.5)
            print("Waiting for camera connection...")
        else:
            print("Camera connected!")
            break

    # 可调外参（被键盘线程直接修改）
    cam_base_link_extrinsics = np.array([
                    [-0.0174, -0.5884,  0.8084,  0.0439],
                    [-0.9998,  0.004,  -0.0186,  0.0443],
                    [ 0.0077, -0.8086, -0.5884,  1.5376],
                    [ 0.,      0.,      0.,      1.    ]
            ])
    rot = cam_base_link_extrinsics[:3, :3]
    translation = cam_base_link_extrinsics[:3, 3].tolist()
    r = R.from_matrix(rot)
    angles = r.as_euler('xyz').tolist()
    # angles = [-2.159999999999998, -0.010000000000000024, -1.5700000000000012]
    # translation = [0.01800000000000007, 0.04799999999999996, 1.5439999999999996]
    param_lock = threading.Lock()

    # 启动键盘监听
    kb = KeyboardListener(angles_ref=angles, trans_ref=translation, lock=param_lock, rot_step=0.01, trans_step=0.002)
    kb.start()

    # 主循环
    try:
        while kb.running:
            start_time = time.time()

            res, point_cloud, _ = orbbec_client.get_pcd_numpy()
            if not res:
                continue
            
            mask = (
                (point_cloud[:,0] >= -2) & (point_cloud[:,0] <= 2) &
                (point_cloud[:,1] >= -2) & (point_cloud[:,1] <= 2) &
                (point_cloud[:,2] >= -0.5) & (point_cloud[:,2] <= 1.0)
            )
            point_cloud = point_cloud[mask]

            with param_lock:
                rot = R.from_euler('xyz', angles)
                rotation_matrix = rot.as_matrix()

                cam_base_link_extrinsics = np.eye(4)
                cam_base_link_extrinsics[:3, :3] = rotation_matrix
                cam_base_link_extrinsics[:3, 3] = translation

            
            point_cloud_h = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
            point_cloud_in_base_link1 = (cam_base_link_extrinsics @ point_cloud_h.T).T[:, :3]

            pcd_msg = numpy_to_pointcloud2(point_cloud_in_base_link1, frame_id="base_link")
            pcd_env_publisher.publish(pcd_msg)

            print(f"Angles: {angles}")
            print(f"translation: {translation}")
            print(f"{cam_base_link_extrinsics}")

            # 控制发布频率（可调）
            time.sleep(0.1)
    finally:
        kb.stop()
        # 等待线程退出并恢复终端设置
        kb.join(timeout=1.0)
# ...existing code...