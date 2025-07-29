"""
from peregrine.pipelines.ros2.utils import ros2_env_setup
ros2_env_setup()
import rclpy
if not rclpy.ok():
    rclpy.init()
"""

import sys
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
sys.path.append('/opt/ros/humble/local/lib/python3.10/dist-packages')

from peregrine.pipelines.ros2.utils import ros2_env_setup
ros2_env_setup()
import rclpy
if not rclpy.ok():
    rclpy.init()

from peregrine.twins import ScenarioScript
class UGVDemo(ScenarioScript):
    def begin_play(self):
        super().begin_play()

    def tick(self, delta_time):
        super().tick(delta_time)

    def end_play(self):
        super().end_play()