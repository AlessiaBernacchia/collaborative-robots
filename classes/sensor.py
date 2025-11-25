import swift
from classes.objects import Brick, Tower
#from classes.robot import Robot 
class Sensor:

    def __init__(self, env: swift.Swift, bricks: list[Brick], towers: list[Tower]):
        self.env = env
        self.bricks = bricks
        self.towers = towers      
    #    self.robots = robots  

    def update(self):
        """
        update the state of the environment
        """
        # TODO by Sensor developer
        pass

    def get_free_bricks(self) -> list[Brick]:
        """
        return the list of free bricks in the environment
        """
        # TODO by Sensor developer
        return self.bricks
    
    def get_towers(self) -> list[Tower]:
        """
        return the list of towers in the environment
        """
        # TODO by Sensor developer
        return self.towers
    
    #def get_robots(self) -> list[Robot]:
    #    """
    #    return the list of robots in the environment
    #    """
    #    return self.robots   
    
    
