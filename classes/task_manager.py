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

# # TODO: in future for task manager
# def execute_stacking_sequence(self, robot, brick, target_pose):
#     # calculate complete path
#     sequence_of_poses = self._calculate_full_path(brick.start_pose, target_pose)
#     # avoid collisions
#     for pose in sequence_of_poses:
#         if self.is_path_safe(robot, pose):
#             robot.move_to_pose(pose)
#         else:
#             print(f"{robot.name}: STOPPED MOVEMENT TO AVOID COLLISION")
#             return
    
#     robot.is_busy = False

# def find_available_robot(self) -> Robot_arm | None:
#     """
#     Find first robot not busy to complete the task
#     """
#     for robot in self.__robots:
#         if not robot.is_busy:
#             robot.is_busy = True
#             return robot
#     return None