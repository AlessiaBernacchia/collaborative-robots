from typing import List, Tuple

from time import sleep
import swift
import spatialmath as sm
import spatialgeometry as sg

import numpy as np
import roboticstoolbox as rtb
import threading

from classes.robot import Robot_arm
from classes.objects import Tower, Brick
from classes.sensor import Sensor
from classes.controller import Controller

# higher it is, higher the velocity of the robots
DT = 0.05

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
    def ask_free_bricks(self) -> List[Brick]:
        """
        ask to the Sensor class a Brick free to pick
        """
        return self._sensor.get_free_bricks()
    
    def ask_available_bricks(self) -> List[Brick]:
        """
        ask to the Sensor class a available Brick to pick
        """
        return self._sensor.get_available_bricks()
    
    def ask_towers(self) -> List[Tower]:
        """
        ask to the Sensor class the towers
        """
        return self._sensor.get_towers()

    def search_uncomplete_tower(self) -> List[Tower] | None:
        """
        search between the towers in the environment one that is incomplete and unlocked
        """
        available_towers = self._sensor.get_available_towers()
        if len(available_towers) == 0:
            return None
        return available_towers[0]
    
    def available_brick(self, robot_name) -> Brick | None:
        """
        return a free and unlocked brick
        """
        # TODO remove panda robot
        free_bricks = self._sensor.get_available_bricks()
        if len(free_bricks) == 0:
            return None
        if robot_name == "panda":
            free_bricks = [b for b in free_bricks if b.pose()[0, 3] <= 0.05]
        elif robot_name == "panda_2":
            free_bricks = [b for b in free_bricks if b.pose()[0, 3] > 0.05]
            
        return free_bricks[0]
        

        
    # internal methods
    def select_free_agent(self) -> Robot_arm | None:
        """
        select a robot not busy to assign a task
        """
        for r in self._robots:
            if not r.is_busy():
                return r
            return None

    def resolve_collision_precedence(self, agent: Robot_arm, agent_error: float):
        """
        Return True if the agent can move safely without causing any collision
        otherwise it return False, causing the retrocession of the agent
        """

        if self._sensor.check_collision(agent):
            ROBOT_WITH_PRECEDENCE_NAME = self._robots[0].name

            robot_target_distances = list(self._sensor.get_current_robot_target_distances())
        
            idx_agent = get_index_by_name(self._robots, agent.name)
            future_robot_distance = agent_error
            other_robots_distances = [d for i, d in enumerate(robot_target_distances) if i != idx_agent]
            min_other_distance = min(other_robots_distances)

            # can move only if 
            # it is the nearest to the target pose 
            if agent.name == ROBOT_WITH_PRECEDENCE_NAME:
                # if more robots has the same error, it has the precedence
                can_move = future_robot_distance <= min_other_distance
            else:
                # has the precedence only if it is the nearest to the target pose
                can_move = future_robot_distance < min_other_distance
            return can_move
        
        # no collision -> safe
        return True

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

        #request a free and unlocked brick
        brick_to_pick = self.available_brick(agent.name)

        # if there is no brick, return nothing
        if brick_to_pick is None:
            return None

        # try to lock the brick
        if not brick_to_pick.try_lock(agent.name):
            return None
        
        # calculate the path points
        way_points_brick = self._controller.generate_path_points(brick_to_pick)

        if way_points_brick is None:
            return None
        
        # 1. go to pick the brick
        # safe pose
        self._controller.move_to_pose(agent, way_points_brick[0], task_manager=self, dt = dt)
        
        # pick the brick
        self._controller.move_to_pose(agent, way_points_brick[1], task_manager=self, dt = dt)
        
        # with the brick, move it to a safe high
        self._controller.move_to_pose(agent, way_points_brick[2], brick=brick_to_pick, task_manager=self, dt = dt)
        
        # search uncompleted and not locked tower
        target_tower = self.search_uncomplete_tower()
        while target_tower is None:
            target_tower = self.search_uncomplete_tower()
            print(f"{agent.name}: can't find available tower")
            sleep(dt*5)
        # try to lock the tower
        if not target_tower.try_lock(agent.name):
            return None
                    
        # 2. move and place the brick        
        way_points = self._controller.generate_path_points(brick_to_pick, target_tower)
        for target_point in way_points[:-1]:
            self._controller.move_to_pose(agent, target_point, brick=brick_to_pick, task_manager=self, dt = dt)

        # fake parallel placing of the brick
        brick_to_pick.placing_orientation()
        # increasing the counter of the bricks on the tower
        target_tower.add(brick_placed=brick_to_pick)
        # flag the brick as placed
        brick_to_pick.placed = True

        # release the locks
        brick_to_pick.unlock(agent.name)
        
        # 3. move to safe height
        self._controller.move_to_pose(agent, way_points[-1], task_manager=self, dt = dt)
        target_tower.unlock(agent.name)
        
        #if self.search_uncomplete_tower() is None or self.ask_free_bricks() is None:
        #    self._controller.rest(agent, self, dt)

        
    

    # parallel methods executions
    def robot_worker(self, robot, max_tasks):
        """
        Worker function that executes tasks for a single robot.
        Each robot will try to complete max_tasks bricks.
        """
        tasks_completed = 0
        
        while tasks_completed < max_tasks:
            robot.start_task()
            result = self.place_one_brick(robot, dt=DT)
            robot.task_completed()
            
            if result is None:  # Task completed successfully
                tasks_completed += 1
            else:
                sleep(DT*30)
            # finish task
            if tasks_completed == max_tasks:
                self._controller.rest(robot, self, dt=DT)
    
    def start(self):        
        # Create two threads, one for each robot
        # Each robot will complete 4 bricks (8 total / 2 robots)
        threads = [
            threading.Thread(
                target=self.robot_worker,
                args=(agent, 4)
            )
            for agent in self._robots]
        
        # Start both robots simultaneously
        for t in threads:
            t.start()
        
        # Wait for both robots to complete their tasks
        for t in threads:
            t.join()
        
        