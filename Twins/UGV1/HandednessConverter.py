# import sys, os
# sys.path.append('/usr/lib/python3/dist-packages')
# sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
# sys.path.append('/opt/ros/humble/local/lib/python3.10/dist-packages')
# os.environ.pop('ROS_DISTRO', None)

from peregrine.pipelines.ros2.utils import ros2_env_setup
ros2_env_setup()
import rclpy
if not rclpy.ok():
    rclpy.init()
from peregrine.twins import SystemTwin
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

import numpy as np

class HandednessConverter(Node):
    def __init__(self):
        super().__init__('handedness_converter')
        
        sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time_param])

        # Subscribers
        self.create_subscription(Imu, '/original_imu', self.imu_callback, 10)
        self.create_subscription(Twist, '/original_imu/twist', self.twist_callback, 10)
        self.create_subscription(Odometry, '/original_imu/odometry', self.odom_callback, 10)
        #self.create_subscription(NavSatFix, '/original_navsatfix', self.navsatfix_callback, 10)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/sensing/imu/imu_raw', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/sensing/imu/imu_raw/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, '/sensing/imu/imu_raw/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/sensing/gnss/pose', 10)
        self.pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', 10)
        self.nav_pub = self.create_publisher(NavSatFix, '/nav_sat_fix', 10)

        self.M = np.diag([1, -1, 1])

    def twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.twist = msg
        twist_stamped.header.frame_id = 'imu_link'
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(twist_stamped)

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = msg.header
        pose_cov.header.frame_id = 'map'
        # pose.header.stamp = msg.header.stamp
        pose_cov.header.stamp = self.get_clock().now().to_msg()
        pose_cov.pose.pose = msg.pose.pose
        covariance = [0.0] * 36
        # covariance[0] = 8.390061017777777
        # covariance[7] = 8.390061017777777
        # covariance[14] = 16.106696004444444
        # covariance[21] = 1.0
        # covariance[28] = 1.0
        # covariance[35] = 1.0
        pose_cov.pose.covariance = covariance

        self.odom_pub.publish(msg)
        self.pose_pub.publish(pose)
        self.pose_cov_pub.publish(pose_cov)

    def imu_callback(self, msg: Imu):
        msg.header.frame_id = 'imu_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = 0.0, 0.0, 0.0, 1.0
        msg.orientation_covariance = [0.0] * 9

        msg.angular_velocity_covariance = [1.15e-5, 0.0, 0.0,
                                           0.0, 1.15e-5, 0.0,
                                           0.0, 0.0, 1.15e-5]

        msg.linear_acceleration_covariance = [1e8, 0.0, 0.0,
                                               0.0, 1e8, 0.0,
                                               0.0, 0.0, 1e8]
        self.imu_pub.publish(msg)

    def navsatfix_callback(self, msg: NavSatFix):
        msg.header.frame_id = 'map' # 'navsat_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.nav_pub.publish(msg)