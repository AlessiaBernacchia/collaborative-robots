class TaskManager:
    def __init__(self, sensor, controller):
        self._sensor = sensor
    
    def decide_and_act(self):
        if self._sensor.check_collision():
            dist_1, dist_2 = self._sensor.robot_dist()
            robot_1, robot_2 = self._sensor.get_robots(self)
            if dist_1 >= dist_2:
                pass
            if dist_1 < dist_2:
                pass