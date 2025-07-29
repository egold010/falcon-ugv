# import sys, os
# sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
# sys.path.append('/opt/ros/humble/local/lib/python3.10/dist-packages')
# os.environ.pop('ROS_DISTRO', None)

from peregrine.pipelines.ros2.utils import ros2_env_setup
# ros2_env_setup(setup_file_linux='/home/evan/autoware/install/setup.bash')
ros2_env_setup()
import rclpy
if not rclpy.ok():
    rclpy.init()
from peregrine.twins import SystemTwin
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

# from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
# from autoware_control_msgs.msg import Control

class ControlSubscriber(Node):
    def __init__(self):
        super().__init__('control_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/control/command/twist_cmd',
            self.control_callback,
            10
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/vehicle_status/falcon/steering_status',
            10
        )
        self.subscription
        self.throttle = 0.0
        self.steering_angle = 0.0
        sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time_param])

        self.active = False

    def control_callback(self, msg):
        self.active = True
        self.throttle = msg.linear.x # m/s^2
        self.steering_angle = msg.angular.z # radians

    def publish_steering_status(self):
        status_msg = PoseStamped()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.pose.orientation.z = self.steering_angle
        self.publisher.publish(status_msg)