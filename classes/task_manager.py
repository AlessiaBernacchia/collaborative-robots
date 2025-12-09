from typing import List, Tuple

import swift
import spatialmath as sm
import spatialgeometry as sg

import numpy as np
import roboticstoolbox as rtb

from classes.robot import Robot_arm
from classes.objects import Tower, Brick
from classes.sensor import Sensor
from classes.controller import Controller

def get_index_by_name(obj_list, target_name):
    """
    Returns the index of the first object in obj_list whose .name matches target_name.
    If not found, returns -1.
    """
    if not isinstance(obj_list, list):
        raise TypeError("First argument must be a list.")

    for idx, obj in enumerate(obj_list):
        # Ensure the object has a 'name' attribute
        if obj.name == target_name:
            return idx
    raise TypeError("Name specified doesn't correspond to any elements.")
class TaskManager:
    def __init__(self, env:swift.Swift, sensor: Sensor, robots: List[Robot_arm], controller: Controller):
        self.__env = env
        self._sensor = sensor
        self._robots = robots
        self._controller = controller
    
    # general Sensor's requests

    def ask_free_bricks(self):# -> Brick:
        """
        ask to the Sensor class a Brick free to pick
        """
        return self._sensor.get_free_bricks()
    
    def ask_towers(self):
        """
        ask to the Sensor class the towers
        """
        return self._sensor.get_towers()

    def search_uncomplete_tower(self):
        """
        search between the towers in the environment one that is incomplete
        """
        towers = self._sensor.get_incomplete_towers()
        return towers[0] if towers else None

    def free_brick(self):
        """
        return a brick free
        """
        free_bricks = self._sensor.get_free_bricks()
        return free_bricks[0] if free_bricks else None
        
    # internal methods
    def select_free_agent(self):
        """
        select a robot not busy to assign a task
        """
        for r in self._robots:
            if not r.is_busy():
                return r
            return None

    def select_brick_and_tower(self, robot_name) -> Tuple[Brick|None, Tower|None]:
        """
        select a free brick and a tower to complete from the sensor environent
        """
        #brick_to_pick: Brick | None = self.free_brick(sensor)
        #target_tower: Tower | None = self.search_uncomplete_tower(sensor)
        free_bricks = self._sensor.get_free_bricks()
        incomplete_towers = self._sensor.get_incomplete_towers()

        # TODO: check distance between robot and brick and pick the nearest one
        if robot_name == "panda":
            free_bricks = [b for b in free_bricks if b.pose()[0, 3] <= 0.05]
        elif robot_name == "panda_2":
            free_bricks = [b for b in free_bricks if b.pose()[0, 3] > 0.05]
    
        # try to lock both resources
        for brick_to_pick in free_bricks:
            if brick_to_pick.try_lock(robot_name):
                for target_tower in incomplete_towers:
                    if target_tower.try_lock(robot_name):
                        return brick_to_pick, target_tower
                # if no target_tower, release brick
                brick_to_pick.unlock(robot_name)
    
        return None, None
        #return brick_to_pick, target_tower

    def place_one_brick(self, agent: Robot_arm, dt=0.01):
        """
        given the agent:
        1. select the robot, the resource to pick and the target
        2. calculates the points to follow to complete the task
        3. moves the arm to make the task
        """
        # 1. select robot, brick and tower
        # select one free robot
        # agent = self.select_free_agent()
        # if agent is None:
        #     return None

        # select one brick and one tower
        brick_to_pick, target_tower = self.select_brick_and_tower(agent.name)

        if brick_to_pick is None or target_tower is None:
            return None
        
        # calculate the path points
        way_points = self._controller.generate_path_points(brick_to_pick, target_tower)

        if way_points is None:
            return None
        
        # 1. go to pick the brick
        # safe pose
        self._controller.move_to_pose(agent, way_points[0], task_manager=self, dt = dt)
        # pick the brick
        self._controller.move_to_pose(agent, way_points[1], task_manager=self, dt = dt)

        # 2. move and place the brick
        for target_point in way_points[2:-1]:
            self._controller.move_to_pose(agent, target_point, brick=brick_to_pick, task_manager=self, dt = dt)

        # fake parallel placing of the brick
        brick_to_pick.placing_orientation()
        # increasing the counter of the bricks on the tower
        target_tower.add(brick_placed=brick_to_pick)
        # flag the brick as placed
        brick_to_pick.placed = True

        # release the locks
        brick_to_pick.unlock(agent.name)
        target_tower.unlock(agent.name)
        
        # 3. move to safe height
        self._controller.move_to_pose(agent, way_points[-1], task_manager=self, dt = dt)
        
        self.__env.step(dt)

        
    def resolve_collision_precedence(self, agent: Robot_arm, agent_error: float):
        """
        Docstring for resolve_collision_precedence
        
        :param self: Description
        :param agent: Description
        :type agent: Robot_arm
        """
        if self._sensor.check_collision():
            ROBOT_WITH_PRECEDENCE_NAME = self._robots[0].name

            robot_target_distances = list(self._sensor.get_current_robot_target_distances())
            # # robot started now
            # if all(robot_target_distances) == np.inf:
            #     if agent.name == ROBOT_WITH_PRECEDENCE_NAME:
            #         return True
            #     return False
            
            if agent.name == ROBOT_WITH_PRECEDENCE_NAME:
                return True
            return False
        
        #     idx_agent = get_index_by_name(self._robots, agent.name)
        #     # robot_distance = robot_target_distances.pop(idx)
        #     future_robot_distance = agent_error
        #     other_robots_distances = [d for i, d in enumerate(robot_target_distances) if i != idx_agent]
        #     min_other_distance = min(other_robots_distances)

        #     # can move only if 
        #     # it is the nearest to the target pose 
        #     if agent.name == ROBOT_WITH_PRECEDENCE_NAME:
        #         # if more robots has the same error, it has the precedence
        #         can_move = future_robot_distance <= min_other_distance
        #     else:
        #         can_move = future_robot_distance < min_other_distance

        #     return can_move
        
        return True

    
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