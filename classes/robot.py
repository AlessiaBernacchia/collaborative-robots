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
        self._ee_pos_hist = []

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
        ee_pos = self._robot.fkine(self._robot.q).t
        self._ee_pos_hist.append(ee_pos)
        
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

    def get_trajectory(self):
        ee = np.array(self._ee_pos_hist)
        t = np.array(self._time_data)
        return ee, t

    def plot_top_view(self):
        ee, _ = self.get_trajectory()

        plt.figure(figsize=(6, 6))
        plt.plot(ee[:,0], ee[:,1], linewidth=2.5)
        plt.scatter(ee[0,0], ee[0,1], c='green', s=120, edgecolor='black', label='Pick')
        plt.scatter(ee[-1,0], ee[-1,1], c='red', s=120, edgecolor='black', label='Place')

        plt.title("Top View (XY)", fontsize=14)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.show()

    def plot_vertical_views(self):
        ee, _ = self.get_trajectory()

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        # Side view (XZ)
        axes[0].plot(ee[:,0], ee[:,2], linewidth=2.5)
        axes[0].axhline(0, linestyle='--', color='brown', alpha=0.6)
        axes[0].set_title("Side View (XZ)")
        axes[0].set_xlabel("X [m]")
        axes[0].set_ylabel("Z [m]")

        # Front view (YZ)
        axes[1].plot(ee[:,1], ee[:,2], linewidth=2.5)
        axes[1].axhline(0, linestyle='--', color='brown', alpha=0.6)
        axes[1].set_title("Front View (YZ)")
        axes[1].set_xlabel("Y [m]")
        axes[1].set_ylabel("Z [m]")

        for ax in axes:
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_height_over_time(self):
        ee, time_data = self.get_trajectory()

        plt.figure(figsize=(10, 5))
        plt.plot(time_data, ee[:,2], linewidth=2.5)
        plt.axhline(0, linestyle='--', color='brown', alpha=0.6, label="Table level")

        for i, t in enumerate(self._tasks_t):
            plt.axvline(t, linestyle='--', color='red', alpha=0.4)

        plt.title("Height Over Time", fontsize=14)
        plt.xlabel("Time [s]")
        plt.ylabel("Z [m]")
        plt.grid(True, alpha=0.3)
        plt.show()

