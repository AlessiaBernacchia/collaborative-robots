from typing import List, Tuple

from time import sleep, time
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
    def __init__(self, env:swift.Swift, sensor: Sensor, robots: List[Robot_arm], controller: Controller, dt: float = 0.05):
        self.__env = env
        self._sensor = sensor
        self._robots = robots
        self._controller = controller

        self.__dt = dt
    
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

    def ask_uncomplete_towers(self) -> List[Tower] | None:
        """
        ask to the Sensor class the incomplete towers
        """
        uncomplete_towers = self._sensor.get_incomplete_towers()
        if len(uncomplete_towers) == 0:
            return None
        return uncomplete_towers
    
    def search_uncomplete_tower(self, brick: Brick = None) -> List[Tower] | None:
        """
        search between the towers in the environment one that is incomplete and unlocked
        """
        brick_color_name = "blue" if brick is None else brick.color_name
        available_towers = self._sensor.get_available_towers()
        if len(available_towers) == 0:
            return None
        
        for tower in available_towers:
            if tower.color_name == brick_color_name:
                return tower
            
        return None 
    
    def _calculate_base_to_brick_distance(self, agent: Robot_arm, brick: Brick) -> float:
        """
        distance btw robot's base and brick pose
        """
        base_pos = agent.base_position
        brick_pos = brick.pose()
        brick_pos = brick_pos[:3, 3]

        base_pos_xy = base_pos[:2]
        brick_pos_xy = brick_pos[:2]       
        
        return np.linalg.norm(base_pos_xy - brick_pos_xy)
    
    def available_brick(self, agent: Robot_arm) -> Brick | None:
        """
        Retrieve the brick highest and nearest to the base of the robot
        the brick to pick and place
        """
        # retrieve free bricks 
        free_bricks = self.ask_available_bricks()
        if not free_bricks:
            return None

        # reachable ones
        reachable_bricks = []
        for brick in free_bricks:
            # select z coordinate of the brick
            brick_z_pos = brick.pose()[2, 3]
            distance = self._calculate_base_to_brick_distance(agent, brick)
            if distance <= agent.max_reach_distance:
                reachable_bricks.append((distance, brick_z_pos, brick))
        
        if not reachable_bricks:
            return None
        
        # sort bricks based on z and nearless
        sorted_bricks = sorted(reachable_bricks, key=lambda x: (-x[1], x[0]))
        # select the highest and nearest one
        min_dist, max_z, nearest_highest_brick = sorted_bricks[0]
        
        return nearest_highest_brick

        
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

        if self._sensor.check_collision():
            print('!! collision detected !!')
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

            # save collision information
            current_time = time()
            distance_between_robots = self._sensor.get_distance_between_robots()
            
            for robot in self._robots:
                had_precedence = (robot.name == agent.name and can_move)
                robot.record_collision_event(
                    timestamp=current_time,
                    position=robot.end_factor_position(),
                    distance=distance_between_robots,
                    had_precedence=had_precedence
                )
                
            return can_move
        
        # no collision -> safe
        return True

    def place_one_brick(self, agent: Robot_arm, dt=0.01):
        """
        Executes the full pick-and-place cycle for a single brick by the given agent.

        This function coordinates resource locking, path planning, low-level movement 
        (via the Controller), and implements task-specific conflict resolution logic.

        Steps:
        1. Resource Acquisition (Brick & Locks):
        - Selects the nearest and highest available brick via self.available_brick(agent).
        - Attempts to lock the selected brick. Fails if lock is unavailable.
        2. Brick Picking Sequence:
        - Moves agent to Safe Pick Pose, then Pick Pose, and lifts the brick to Safe High.
        - NOTE: Collision precedence is checked by the TaskManager at every move step.
        3. Target Acquisition and Synchronization:
        - Searches for an available, incomplete Tower.
        - Enters a loop: If no tower is available, agent waits briefly.
        - Fallback Logic: If no towers become available before the project completes, 
            the agent executes a safe drop of the brick and enters the rest pose.
        - Attempts to lock the target tower.
        4. Placing Sequence:
        - Moves the agent (with the brick) through the calculated path points to the target pose.
        - Finalizes brick orientation and adds it to the Tower stack.
        - Releases locks on the Brick and Tower resources.
        5. Cleanup:
        - Moves the agent to the final Safe Height position.
        
        Args:
            agent (Robot_arm): The robot assigned to perform the task.
            dt (float): Time step increment for the simulation environment.
            
        Returns:
            None: If the task was completed successfully.
            Any: If the task failed due to missing/unavailable resources.
        """

        #request a free and unlocked brick
        brick_to_pick = self.available_brick(agent)

        # if there is no brick, return nothing
        if brick_to_pick is None:
            print(f'{agent.name} : no bricks to pick available')
            return None

        # try to lock the brick
        if not brick_to_pick.try_lock(agent.name):
            print(f'{agent.name} : error, brick not lockable')
            return None
        
        # calculate the path points
        way_points_brick = self._controller.generate_path_points(brick_to_pick)

        if way_points_brick is None:
            print(f'{agent.name} : error, no way points')
            return None
        
        # 1. go to pick the brick
        # safe pose
        self._controller.move_to_pose(agent, way_points_brick[0], task_manager=self, dt = dt)
        
        # pick the brick
        self._controller.move_to_pose(agent, way_points_brick[1], task_manager=self, dt = dt)
        
        # with the brick, move it to a safe high
        self._controller.move_to_pose(agent, way_points_brick[2], brick=brick_to_pick, task_manager=self, dt = dt)
        
        # search uncompleted and not locked tower
        target_tower = self.search_uncomplete_tower(brick_to_pick)
        while target_tower is None:
            target_tower = self.search_uncomplete_tower(brick_to_pick)
            print(f"{agent.name}: can't find available tower, retry")
            sleep(dt*5)

            if not self.has_work_remaining(agent):
                # get down the brick
                print(f'{agent.name} : No tower or free bricks, leave down the brick')
                self._controller.move_to_pose(agent, way_points_brick[1], brick=brick_to_pick, task_manager=self, dt = dt)
                # fake parallel placing of the brick
                brick_to_pick.placing_orientation()
                # release the locks
                brick_to_pick.unlock(agent.name)

                self._controller.rest(agent, self, dt=dt)
                print(f"{agent.name} : No work remaining, let's go to rest")
                return None
            
        # try to lock the tower
        if not target_tower.try_lock(agent.name):
            print(f'{agent.name} : error, tower not lockable')
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
        print(f'{agent.name} : BRICK PLACED')

        
    # parallel methods executions
    def has_work_remaining(self, agent: Robot_arm) -> bool:
        """
        verify whether the task is complete and there is work remaining or not
        """
        #has_bricks = self.ask_free_bricks() is not None
        has_towers = self.ask_uncomplete_towers() is not None

        #has_available_towers = self.search_uncomplete_tower() is not None
        has_available_bricks = self.available_brick(agent) is not None

        if has_available_bricks and not has_towers:
            # there are bricks, but all towers are complete
            return False
        
        if not has_available_bricks:
            # there no bricks free
            return False
        return True
    
    def robot_worker(self, robot: Robot_arm):
        """
        Worker function that executes tasks for a single robot.
        Each robot will work until there are resources or targets.
        """
        
        while self.has_work_remaining(robot):
            robot.start_task()
            result = self.place_one_brick(robot, dt=self.__dt)
            robot.task_completed()
            
            if result is not None:  # Task not completed -> retry
                # wait a moment before recheck the resources available
                sleep(self.__dt*30)
                print(f'{robot.name} : Still be work to do, but not reachable resources')
            # finish task

        # when it finish the tasks -> rest pose
        self._controller.rest(robot, self, dt=self.__dt)
        print(f'{robot.name} : I have finished here. Goodbye!!')
    
    def start(self):        
        # Create two threads, one for each robot
        # Each robot will complete 4 bricks (8 total / 2 robots)
        threads = [
            threading.Thread(
                target=self.robot_worker,
                args=(agent,)
            )
            for agent in self._robots]
        
        # Start both robots simultaneously
        for t in threads:
            t.start()
        
        # Wait for both robots to complete their tasks
        for t in threads:
            t.join()