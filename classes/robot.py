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
        self._qd_hist = []
        self._cond_hist = []
        
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
        
    def apply_velocity_cmd(self, qdot: np.ndarray, cond_number):
        """
        apply the modification of the joints to the robot
        """
        self._robot.qd = qdot
        self._qd_hist.append(qdot)
        self._cond_hist.append(cond_number)
    
    def plot_metrics(self, dt):
        fig = plt.figure('Joint positions')
        time_data = np.arange(0,len(self._qd_hist))*dt
        plt.xlim(0, time_data[-1])
        plt.grid(True)
        plt.plot(time_data,self._qd_hist,'k')
        plt.plot(time_data,self._cond_hist,'r--')
        plt.title('Joint positions')
        plt.xlabel('t [s]')
        plt.ylabel('q_i [rad]')
        plt.grid(color='0.95')
        plt.show()

        


