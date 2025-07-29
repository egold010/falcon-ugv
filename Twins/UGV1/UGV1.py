from peregrine.twins import SystemTwin
import os
import dupy_unreal as dupy
from peregrine.conf import global_settings
from peregrine.sensors.capture_sensor import PointCloud
global_settings.Settings.BASE_OUTPUT_DIRECTORY = os.path.dirname(dupy.find_object(name=u'DuRTLFunctionLibrary', _class=dupy.find_object(name=u'Class')).get_cdo().call_function("GetCurrentScenarioFilename")) + "/Output"

from ControlSubscriber import *
from PointCloudPublisher import *
from HandednessConverter import *

class UGV1(SystemTwin):
    def begin_play(self):
        super().begin_play()
        self.sensor_manager.start()
        self.control_subscriber = ControlSubscriber()
        self.lidar_publisher = LidarPublisher()
        self.handedness_converter = HandednessConverter()
        # rclpy.spin(self.clock_pub)
        self.vehicle_movement_component = self.get_property("VehicleMovementComponent")

    def tick(self, delta_time):
        super().tick(delta_time)
        system = dupy.get_simulation_world().get_player_pawn()
        self.sensor_manager.capture_and_run_pipelines(self.sim_time)
        points_scope = system.get_child_scope("CaptureSensorLidar_BP_C_0/CaptureSensorLidar_BP_C_0_WorldPosition") 
        point_cloud: PointCloud = system.sensor_manager.get_sensor_data(points_scope)
        self.lidar_publisher.process_lidar_data(point_cloud)
        self.control_subscriber.publish_steering_status()

        rclpy.spin_once(self.lidar_publisher, timeout_sec=0.01)
        rclpy.spin_once(self.handedness_converter, timeout_sec=0.01)
        rclpy.spin_once(self.control_subscriber, timeout_sec=0.01)

        if self.control_subscriber.active:
            system.set_property('Autonomous', True)
            print("throttle: ", self.control_subscriber.throttle, "steering angle: ", self.control_subscriber.steering_angle)
            self.set_throttle_input(self.control_subscriber.throttle)
            self.set_steering_input(self.control_subscriber.steering_angle)

    def end_play(self):
        super().end_play()
        self.control_subscriber.destroy_node()
        self.lidar_publisher.destroy_node()
        self.handedness_converter.destroy_node()
        self.sensor_manager.stop()

    def set_throttle_input(self, throttle: float):
        if throttle > 0.2:
            self.vehicle_movement_component.call_function("SetThrottleInput", 1.0)
            self.vehicle_movement_component.call_function("SetBrakeInput", 0.0)
        elif throttle < -0.2:
            self.vehicle_movement_component.call_function("SetThrottleInput", 0.0)
            if throttle < -3:
                self.vehicle_movement_component.call_function("SetBrakeInput", 1.0)
        else:
            self.vehicle_movement_component.call_function("SetThrottleInput", 0.0)
    
    def set_steering_input(self, steer: float):
        self.vehicle_movement_component.call_function("SetSteeringInput", -steer / 0.4) # expects -1 to 1 steering input. 0.4 is max for vehicle spec