from peregrine.twins import SystemTwin
import os
import dupy_unreal as dupy
from peregrine.conf import global_settings
global_settings.Settings.BASE_OUTPUT_DIRECTORY = os.path.dirname(dupy.find_object(name=u'DuRTLFunctionLibrary', _class=dupy.find_object(name=u'Class')).get_cdo().call_function("GetCurrentScenarioFilename")) + "/Output"

class LidarStick(SystemTwin):
    def begin_play(self):
        super().begin_play()
        self.sensor_manager.start()

        self.speed = 1000
        self.rotation_speed = 100
    
    def tick(self, delta_time):
        super().tick(delta_time)
        self.sensor_manager.capture_and_run_pipelines(self.sim_time)

        forward = self.GetActorForwardVector()

        forward_movement = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        forward_movement["X"] = self.input.get_analog("LeftStickYAxis") * delta_time*self.speed * forward["X"]
        forward_movement["Y"] = self.input.get_analog("LeftStickYAxis") * delta_time*self.speed * forward["Y"]
        forward_movement["Z"] = self.input.get_analog("LeftStickYAxis") * delta_time*self.speed * forward["Z"]

        location = self.K2_GetActorLocation()
        location["X"] += forward_movement["X"]
        location["Y"] += forward_movement["Y"]
        location["Z"] += forward_movement["Z"]

        rot = self.K2_GetActorRotation()
        rot["Yaw"] += self.input.get_analog("LeftStickXAxis") * delta_time * self.rotation_speed

        self.K2_SetActorLocation(location)
        self.K2_SetActorRotation(rot)

    def end_play(self):
        super().end_play()