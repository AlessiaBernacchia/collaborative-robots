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
        self._busy = False

        # q dot history
        self._qd_hist = []
        # condition number history
        self._cond_hist = []
        # end effector positions history
        self._ee_pos_hist = []

        self._start_time_tasks = []
        self._end_time_tasks = []
        self._time_data = []

        self._tasks_t = [] 
        self._start_time = 0

    def is_busy(self) -> bool:
        """
        return the state of the robot,
        if it's busy in a task -> True
        if it's free -> False
        """
        return self._busy
    
    def update_current_state(self, cond_number: float):
        """
        update the histories: qdot, conditional number and end factor positions
        safe the time in the internal list
        """
        t = time()
        self.keep_time(t)

        self._qd_hist.append(self._robot.qd)
        self._cond_hist.append(cond_number)

        # ee_pos = self._robot.fkine(self._robot.q).t
        self._ee_pos_hist.append(self.end_factor_position())

    
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
        # return np.asarray(self._robot.q, dtype=float)
        return self._robot.fkine(self._robot.q).t
        
    def apply_velocity_cmd(self, qdot: np.ndarray, cond_number: float):
        """
        apply the modification of the joints to the robot
        """
        self._robot.qd = qdot
        self.update_current_state(cond_number)
        
    def start_task(self, start):
        """
        robot starts a task and its state is busy
        """
        # self._start_time = start
        # self._tasks_t.append(start)
        # self._start_task_times.append(start)
        self._busy = True
        self._start_time_tasks.append(time())
        
    def keep_time(self, t):
        """
        save the time step
        """
        self._time_data.append(t)
    
    def task_completed(self, t):
        """
        robot completed the task and its state return free
        """
        #self._tasks_t.append(t)
        self._end_time_tasks.append(t)
        self._busy = False

    # TODO: quando faremo più tasks non va bene salvare lo start_time
    # bisogna salvare i tempi in modo 'sporco', senza -start_time
    # e 'pulirli' solo quando si plotta in base alla task a cui si fa riferimento
    # quindi tenere le due liste self._time_data e sels._tasks_t (che tiene i tempi limiti delle tasks, quindi i futuri possibili start_time in base alla task di riferimento)
    # o due liste: start_tasks_time e end_tasks_time
    
    def plot_metrics(self, window_tasks=None):
        if window_tasks is None:
            window_tasks = [0, len(self._start_time_tasks)]
        
        idx_first_task = window_tasks[0]
        idx_last_task = window_tasks[-1]

        if idx_last_task >= len(self._start_time_tasks):
            idx_last_task = len(self._start_time_tasks) - 1

        start_time_abs = self._start_time_tasks[idx_first_task]

        if idx_last_task < len(self._end_time_tasks):
            end_time_abs = self._end_time_tasks[idx_last_task]
        else:
            end_time_abs = self._time_data[-1] if self._time_data else start_time_abs

        time_data_np = np.array(self._time_data)

        # find index of the first temporal point >= start_time_abs
        start_idx = np.searchsorted(time_data_np, start_time_abs, side='left')
        # find index of the first temporal point >= end_time_abs
        end_idx = np.searchsorted(time_data_np, end_time_abs, side='right')

        # scale data
        time_data_plot = time_data_np[start_idx:end_idx] - start_time_abs
        # filter data
        qd_hist_plot = np.array(self._qd_hist)[start_idx:end_idx]
        cond_hist_plot = np.array(self._cond_hist)[start_idx:end_idx]
        
        if len(time_data_plot) == 0:
            print("No data in the specified tasks window")
            return
        start_time = self._start_time_tasks[idx_first_task]
        time_data = map(lambda t: t-start_time, np.array(self._time_data))
        fig = plt.figure('Joint positions')
        
        # time_data = np.array(self._time_data)
        plt.xlim(0, time_data[-1])
        plt.grid(True)
        plt.plot(time_data,qd_hist_plot,'k')
        plt.plot(time_data,cond_hist_plot,'b--')
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

