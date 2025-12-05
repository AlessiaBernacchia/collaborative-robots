import swift
from classes.objects import Brick, Tower
from classes.robot import Robot_arm
import numpy as np
class Sensor:

    def __init__(self, env: swift.Swift, bricks: list[Brick], towers: list[Tower], robots: list):
        self.env = env
        self.bricks = bricks
        self.towers = towers      
        self.robots = robots  

    def get_robots(self) -> list[Robot_arm]:
        """
        return the list of robots in the environment
        """
        return self.robots 

    def get_free_bricks(self) -> list[Brick]:
        """
        return the list of free bricks in the environment
        """
        return [brick for brick in self.bricks if not brick.placed]
    
    def get_towers(self) -> list[Tower]:
        """
        return the list of towers in the environment
        """
        return self.towers
    
    def get_incomplete_towers(self) -> list[Tower]:
        """
        return the list of incomplete towers in the environment
        """
        return [tower for tower in self.towers if not tower.is_complete()]
    
    def update(self):
        """
        update the state of the environment
        """
        for brick in self.bricks:
            brick.current_pose = brick.obj.T

        # here we could also update robot current poses to control the distances and avoid collisions
        # for robot in self.robots:
        #     robot.current_pose = robot._robot.fkine(robot.q)

    def check_collision(self):
        """
        check if the two robots' end-factors are in a 
        dangerous distance between each others
        """
        distance = np.linalg.norm(self.robots[0].end_factor_position()-self.robots[1].end_factor_position())
        if distance <= 0.15:
            return True
        return False
    
    def robot_distance(self):
        return self.robots[0].distance, self.robots[1].distance

        
    
    
