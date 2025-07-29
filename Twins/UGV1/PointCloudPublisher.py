# import sys, os
# sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
# sys.path.append('/opt/ros/humble/local/lib/python3.10/dist-packages')

from peregrine.pipelines.ros2.utils import ros2_env_setup
ros2_env_setup()
import rclpy
if not rclpy.ok():
    rclpy.init()
from peregrine.twins import SystemTwin
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time_param])

        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )

    def process_lidar_data(self, point_cloud):
        if not point_cloud.ready: return
        
        # flatten
        n_az = point_cloud.azimuth_samples
        n_el = point_cloud.elevation_samples
        xyz = point_cloud.data.reshape(-1, 3)
        N   = xyz.shape[0]

        # ring indexes for each point
        channel = np.tile(
            np.arange(n_el, dtype=np.uint16),
            reps=n_az
        ).reshape(-1)

        # point dtype
        dtype = np.dtype([
            ('x',          np.float32),
            ('y',          np.float32),
            ('z',          np.float32),
            ('intensity',  np.uint8),
            ('return_type',np.uint8),
            ('channel',    np.uint16),
        ])

        # allocate and populate
        cloud_arr = np.zeros(N, dtype=dtype)
        cloud_arr['x']           = xyz[:, 0] / 100
        cloud_arr['y']           = -xyz[:, 1] / 100
        cloud_arr['z']           = xyz[:, 2] / 100
        cloud_arr['intensity']   = 1.0            # always 1
        cloud_arr['return_type'] = 0.0            # always 0
        cloud_arr['channel']     = channel        # ring index

        hdr = Header()
        hdr.stamp    = self.get_clock().now().to_msg()
        hdr.frame_id = 'lidar_link' # 'lidar_link'  # change to your TF frame

        # defile fields
        fields = [
            PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',   offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='channel',     offset=14, datatype=PointField.UINT16, count=1),
        ]

        # pack into a PointCloud2
        cloud_msg = pc2.create_cloud(
            header=hdr,
            fields=fields,
            points=cloud_arr
        )

        self.pc_pub.publish(cloud_msg)