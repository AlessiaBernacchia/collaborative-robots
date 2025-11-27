import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift
import matplotlib as mtb
import matplotlib.pyplot as plt

class Robot_arm:
    def __init__(self, name:str):
        self.name = name
        self._robot = rtb.models.Panda()
        self._robot.q = self._robot.qr
        
    def register(self, env: swift.Swift):
        """
        add the robot to the enviroment
        """
        env.add(self._robot)
        
    def set_position(self, x=0, y=0, z=0):
        """
        set the position of the base in the enviroment
        """
        self._robot.base = sm.SE3(x, y, z)
    
    def end_factor_position(self):
        """
        give back the end-factor position
        """
        return np.asarray(self._robot.q, dtype=float)
        
    def apply_velocity_cmd(self, qdot: np.ndarray):
        """
        apply the modification of the joints to the robot
        """
        self._robot.qd = qdot
    
    def modify_orientation_base(self):
        """
        modify the orientation of the robot base
        """
        self._robot.base *= sm.SE3.Rz(np.pi)
        


