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
        
    def modify_orientation_base(self):
        """
        modify the orientation of the robot base
        """
        self._robot.base *= sm.SE3.Rz(np.pi)
    
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
        
    def start_task(self):
        """
        Records the absolute start time of a new task.
        """
        self._start_time_tasks.append(time())
        self._busy = True
        
    def keep_time(self, t):
        """
        save the time step
        """
        self._time_data.append(t)
    
    def task_completed(self):
        """
        Records the absolute end time of the task.
        """
        #self._tasks_t.append(t)
        self._end_time_tasks.append(time())
        self._busy = False

    # TODO: quando faremo più tasks non va bene salvare lo start_time
    # bisogna salvare i tempi in modo 'sporco', senza -start_time
    # e 'pulirli' solo quando si plotta in base alla task a cui si fa riferimento
    # quindi tenere le due liste self._time_data e sels._tasks_t (che tiene i tempi limiti delle tasks, quindi i futuri possibili start_time in base alla task di riferimento)
    # o due liste: start_tasks_time e end_tasks_time
    
    def plot_performance_metrics(self, window_tasks=None):
        """
        Plots the robot's control metrics: Joint Velocities (q_dot) and 
        Jacobian Condition Number, filtered by a specified task window.
        
        Args:
            window_tasks (list, optional): [start_task_index, end_task_index]
                                            to filter the plotted data. Defaults to all tasks.
        """
        if window_tasks is None:
            window_tasks = [0, len(self._start_time_tasks)]
        
        idx_first_task = window_tasks[0]
        idx_last_task = window_tasks[-1]

        if idx_last_task >= len(self._start_time_tasks):
            idx_last_task = len(self._start_time_tasks) - 1

        # determine the absolute start and end times for the window
        start_time_abs = self._start_time_tasks[idx_first_task]

        if idx_last_task < len(self._end_time_tasks):
            end_time_abs = self._end_time_tasks[idx_last_task]
        else:
            # if the last task end time is not available, use the last logged time
            end_time_abs = self._time_data[-1] if self._time_data else start_time_abs

        time_data_np = np.array(self._time_data)

        # find index of the first temporal point >= start_time_abs
        start_idx = np.searchsorted(time_data_np, start_time_abs, side='left')
        # find index of the first temporal point >= end_time_abs
        end_idx = np.searchsorted(time_data_np, end_time_abs, side='right')

        # filter and scale time data relative to the start of the window
        time_data_plot = time_data_np[start_idx:end_idx] - start_time_abs
        # filter data
        qd_hist_plot = np.array(self._qd_hist)[start_idx:end_idx]
        cond_hist_plot = np.array(self._cond_hist)[start_idx:end_idx]
        
        if len(time_data_plot) == 0:
            print("No data in the specified tasks window")
            return
        
        # prepare task vertical lines, scaled to the plot window
        task_markers = [t - start_time_abs for t in self._end_time_tasks if start_time_abs <= t <= end_time_abs]

        # start_time = self._start_time_tasks[idx_first_task]
        # time_data = map(lambda t: t-start_time, np.array(self._time_data))
        
        fig, ax = plt.subplots(2, 1, figsize=(10, 4))
        #plt.figure('Robot Control Performance Metrics')
        
        # time_data = np.array(self._time_data)
        ax[0].set_xlim(0, time_data_plot[-1])
        ax[1].set_xlim(0, time_data_plot[-1])
        ax[0].grid(True)
        ax[1].grid(True)
        ax[0].plot(time_data_plot,qd_hist_plot,'k',label=r'Joint Velocities ($\dot{q}$)')
        ax[1].plot(time_data_plot,cond_hist_plot,'b--',label=r'Jacobian Condition Number ($\kappa$)')

        has_label = False
        for t in task_markers:
            label = "Task End Boundary" if not has_label else None
            ax[0].axvline(x=t, linestyle='--', color='red', alpha=0.4, label=label)
            ax[1].axvline(x=t, linestyle='--', color='red', alpha=0.4, label=label)

            has_label = True

        ax[0].set_title('Robot Control Performance Metrics')
        #.title('Robot Control Performance Metrics - joint positions')
        ax[1].set_xlabel('Time [s]')
        ax[1].set_ylabel(r'Condition number')
        ax[0].set_ylabel(r'$q_i [rad]$')
        
        plt.grid(True, alpha=0.5, color='0.95')
        #ax[0].legend()
        #ax[1].legend()
        plt.show()

    def get_trajectory(self):
        """
        Retrieves the history of end-effector positions and corresponding time data.

        Returns:
            tuple[np.ndarray, np.ndarray]: (ee_positions, time_data)
        """
        ee = np.array(self._ee_pos_hist)
        t = np.array(self._time_data)
        return ee, t

    def plot_3d_trajectory_views(self):
        """
        Plots the robot's end-effector trajectory in three orthogonal 2D views 
        (Top, Side, Front) on a single figure.
        """
        ee, _ = self.get_trajectory()

        if len(ee) == 0:
            print("No trajectory data available to plot.")
            return

        fig, axes = plt.subplots(1, 3, figsize=(10, 4))
        
        # Common settings
        start_pos = ee[0]
        end_pos = ee[-1]
        
        # 1. Top View (XY)
        axes[0].plot(ee[:,0], ee[:,1], linewidth=1.5, color='blue', alpha=0.5)
        # axes[0].scatter(start_pos[0], start_pos[1], c='green', s=120, edgecolor='black', label='Start/Pick')
        # axes[0].scatter(end_pos[0], end_pos[1], c='red', s=120, edgecolor='black', label='End/Place')
        axes[0].set_title("Top View (XY Plane)")
        axes[0].set_xlabel("X [m]")
        axes[0].set_ylabel("Y [m]")
        axes[0].axis("equal")
        
        # 2. Side View (XZ Plane)
        axes[1].plot(ee[:,0], ee[:,2], linewidth=1.5, color='blue', alpha=0.5)
        axes[1].axhline(0, linestyle='--', color='brown', alpha=0.6, label="Base Level")
        axes[1].set_title("Side View (XZ Plane)")
        axes[1].set_xlabel("X [m]")
        axes[1].set_ylabel("Z [m]")
        axes[1].axis("equal")
        
        # 3. Front View (YZ Plane)
        axes[2].plot(ee[:,1], ee[:,2], linewidth=1.5, color='blue', alpha=0.5)
        axes[2].axhline(0, linestyle='--', color='brown', alpha=0.6, label="Base Level")
        axes[2].set_title("Front View (YZ Plane)")
        axes[2].set_xlabel("Y [m]")
        axes[2].set_ylabel("Z [m]")
        axes[2].axis("equal")
        
        # Apply common styling
        for ax in axes:
            ax.grid(True, alpha=0.3)
            
        fig.suptitle('End-Effector Trajectory in 3D Views', fontsize=12)
        plt.tight_layout(rect=[0, 0, 1, 0.95]) # Adjust layout to fit suptitle
        plt.show()

    def plot_top_view(self):
        """
        Plots the end-effector's vertical position (Z-coordinate) as a function of time,
        showing task separation markers.
        """
        ee, _ = self.get_trajectory()

        if len(ee) == 0:
            print("No trajectory data available to plot.")
            return

        plt.figure('Height Over Time', figsize=(6, 6))
        plt.plot(ee[:,0], ee[:,1], linewidth=1.5)
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
        axes[0].plot(ee[:,0], ee[:,2], linewidth=1.5)
        axes[0].axhline(0, linestyle='--', color='brown', alpha=0.6)
        axes[0].set_title("Side View (XZ)")
        axes[0].set_xlabel("X [m]")
        axes[0].set_ylabel("Z [m]")

        # Front view (YZ)
        axes[1].plot(ee[:,1], ee[:,2], linewidth=1.5)
        axes[1].axhline(0, linestyle='--', color='brown', alpha=0.6)
        axes[1].set_title("Front View (YZ)")
        axes[1].set_xlabel("Y [m]")
        axes[1].set_ylabel("Z [m]")

        for ax in axes:
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_height_over_time(self, window_tasks=None):
        """
        Plots the end-effector's vertical position (Z-coordinate) as a function of time,
        using relative time and showing task separation markers, filtered by task window.
        
        Args:
            window_tasks (list, optional): [start_task_index, end_task_index] 
                                        to filter the plotted data. Defaults to all tasks.
        """
        # retrieve relatives times

        if window_tasks is None:
            window_tasks = [0, len(self._start_time_tasks)]
        
        idx_first_task = window_tasks[0]
        idx_last_task = window_tasks[-1]

        if idx_last_task >= len(self._start_time_tasks):
            idx_last_task = len(self._start_time_tasks) - 1

        # get absolute start and end times
        try:
            start_time_abs = self._start_time_tasks[idx_first_task]
        except IndexError:
            print("Error: Task window indices are out of range for start times.")
            return

        if idx_last_task < len(self._end_time_tasks):
            end_time_abs = self._end_time_tasks[idx_last_task]
        else:
            end_time_abs = self._time_data[-1] if self._time_data else start_time_abs

        time_data_np = np.array(self._time_data)
        ee, _ = self.get_trajectory() # Get full trajectory data

        # find indices for filtering
        start_idx = np.searchsorted(time_data_np, start_time_abs, side='left')
        end_idx = np.searchsorted(time_data_np, end_time_abs, side='right')

        # filter and scale data
        time_data_plot = time_data_np[start_idx:end_idx] - start_time_abs
        ee_z_plot = ee[start_idx:end_idx, 2]
        
        if len(time_data_plot) == 0:
            print("No data in the specified tasks window.")
            return

        # prepare task vertical lines 
        # only in thw window
        task_end_markers = [
            t - start_time_abs for t in self._end_time_tasks 
            if start_time_abs <= t <= end_time_abs
        ]
        
        # plotting

        plt.figure('Height Over Time', figsize=(10, 5))
        
        # height wrt Z
        plt.plot(time_data_plot, ee_z_plot, linewidth=1.5, color='purple')
        plt.axhline(0, linestyle='--', color='brown', alpha=0.6, label="Table Level")

        # Aggiunge le linee verticali di separazione delle attività
        # flag to add label once in the legend
        has_label = False
        for t in task_end_markers:
            label = "Task End Boundary" if not has_label else None
            plt.axvline(t, linestyle='--', color='red', alpha=0.4, label=label)
            has_label = True

        plt.title("End-Effector Height (Z) Over Relative Time", fontsize=14)
        plt.xlabel("Time [s] (Relative to Task Start)")
        plt.ylabel("Z Position [m]")
        plt.xlim(0, time_data_plot[-1]) 
        
        plt.grid(True, alpha=0.3)
        plt.legend(loc='best')
        plt.show()