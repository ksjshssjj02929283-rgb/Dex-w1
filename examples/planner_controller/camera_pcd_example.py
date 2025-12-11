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

def numpy_to_pointcloud2(points: np.ndarray, frame_id="base_link") -> PointCloud2:
    """
    numpy points: shape (N, 3), dtype float32
    """
    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = frame_id

    return point_cloud2.create_cloud_xyz32(header, points.tolist())

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

    angles = [-0.1, np.pi/2-0.1, -0.05 - 0.1]  
    rot = R.from_euler('zyz', angles)
    rotation_matrix = rot.as_matrix()

    cam_base_link_extrinsics = np.eye(4)
    cam_base_link_extrinsics[:3, :3] = rotation_matrix
    cam_base_link_extrinsics[:3, 3] = [0.01 - 0.01, -0.1605, 0.02 - 0.01]

    # base_link_T_neck2 = np.array([
    #     [ 0.3381, -0.8701, -0.3587,  0.208 ],
    #    [ 0.2795, -0.2711,  0.9211,  0.    ],
    #    [-0.8986, -0.4116,  0.1516,  1.2791],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]
    # ])
    # waist_T_neck2 = np.array([
    #     [ 0.6613, -0.6412, -0.3894,  0.    ],
    #    [ 0.2795, -0.2711,  0.9211,  0.    ],
    #    [-0.6961, -0.7179,  0.    ,  0.389 ],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]
    # ])
    # base_link_T_waist = np.array([
    #     [ 0.9211,  0.    ,  0.3893,  0.0566],
    #    [-0.    ,  1.    , -0.    ,  0.    ],
    #    [-0.3893, -0.    ,  0.9211,  0.9207],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]
    # ])

    while True:
        start_time = time.time()

        res, point_cloud, _ = orbbec_client.get_pcd_numpy()
        # print(f"get pcd time: {time.time() - start_time:.6f}s")

        if not res:
            continue
        
        mask = (
            (point_cloud[:,0] >= -2) & (point_cloud[:,0] <= 2) &
            (point_cloud[:,1] >= -2) & (point_cloud[:,1] <= 2) &
            (point_cloud[:,2] >= -0.5) & (point_cloud[:,2] <= 1.2)
        )
        point_cloud = point_cloud[mask]

        point_cloud_h = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
        point_cloud_in_base_link1 = (cam_base_link_extrinsics @ point_cloud_h.T).T[:, :3]
        # point_cloud_in_base_link2 = (base_link_T_neck2 @ cam_neck2_extrinsics @ point_cloud_h.T).T[:, :3]

        pcd_msg = numpy_to_pointcloud2(point_cloud_in_base_link1, frame_id="base_link")
        # print(f"processing time: {time.time() - start_time:.6f}s")
        pcd_env_publisher.publish(pcd_msg)

        # time.sleep(0.05)