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
        env.add(self._robot)
        
    def set_position(self, x=0, y=0, z=0):
        self._robot.base = sm.SE3(x, y, z)
    
    def end_factor_position(self):
        return np.asarray(self._robot.q, dtype=float)
        
    def apply_velocity_cmd(self, qdot: np.ndarray):
        self._robot.qd = qdot

        
if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    panda_agent = Robot_arm("panda")
    panda_agent.set_position(-0.5, 0, 0)
    panda_agent.register(env)
    

        


