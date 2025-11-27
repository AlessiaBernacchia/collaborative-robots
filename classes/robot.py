import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift
import matplotlib as mtb
import matplotlib.pyplot as plt
from time import time 

class Robot_arm:
    def __init__(self, name:str):
        self.name = name
        self._robot = rtb.models.Panda()
        self._robot.q = self._robot.qr
        self._qd_hist = []
        self._cond_hist = []
        self._tasks_t = [] 
        self._time_data = []
        self._start_time = 0
        
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
        start = time()
        self._robot.qd = qdot
        self._qd_hist.append(qdot)
        self._cond_hist.append(cond_number)
        self.keep_time(start)
        
    def start_time(self, start):
        self._start_time = start
        
    def keep_time(self, t):
        self._time_data.append(t-self._start_time)
    
    def task_completed(self, t):
        self._tasks_t.append(t)
    
    def plot_metrics(self):
        fig = plt.figure('Joint positions')
        time_data = np.array(self._time_data)
        plt.xlim(0, time_data[-1])
        plt.grid(True)
        plt.plot(time_data,self._qd_hist,'k')
        plt.plot(time_data,self._cond_hist,'b--')
        for t in self._tasks_t:
            plt.axvline(x=t, color='red', linestyle='--')
        plt.title('Joint positions')
        plt.xlabel('t [s]')
        plt.ylabel('q_i [rad]')
        plt.grid(color='0.95')
        plt.show()

        


