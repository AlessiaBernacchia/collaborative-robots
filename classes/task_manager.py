class TaskManager:
    def __init__(self, sensor):
        self._sensor = sensor
    
    def decide_and_act(self):
        if self._sensor.check_collision():
            dist_1, dist_2 = self._sensor.robot_dist()
            if dist_1 >= dist_2:
                pass