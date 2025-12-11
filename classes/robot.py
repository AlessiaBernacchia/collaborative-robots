import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import swift
import matplotlib as mtb
import matplotlib.pyplot as plt
from time import time 

class Robot_arm:
    def __init__(self, name:str, frame=sm.SE3.Rz(0), safe_transition=sm.SE3.Tx(-0.2), max_reach_distance: float=0.90):
        self.name = name
        self._robot = rtb.models.Panda()
        self._robot.q = self._robot.qr
        
        self._safe_transition = safe_transition
        self._transform = frame

        self._max_reach_distance = max_reach_distance

        self._distance = np.inf
        self._busy = False
        self._position = None
        self._continue = True

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

        #tracking collision
        self._collision = []

    def is_busy(self) -> bool:
        """
        return the state of the robot,
        if it's busy in a task -> True
        if it's free -> False
        """
        return self._busy
    
    def rest_qr(self):
        """
        return the joint positions of the rest position
        """
        return self._robot.qr
    
    @property
    def max_reach_distance(self):
        """
        Maximum distance from the base the robot is capable to reach with its end effector
        """
        return self._max_reach_distance
    
    @property 
    def distance(self):
        """
        getter function that return the distance
        between the end-factor and the target
        """
        return self._distance
    
    @property
    def base_position(self):
        """
        getter function that return the position of the base
        """
        if self._position is None:
            return self._robot.base.t
        return self._position.t
    
    def reset_distance(self):
        """
        reset the distance
        """
        self._distance = np.inf

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
        self._position = sm.SE3(x, y, z)
        
    def modify_orientation_base(self):
        """
        modify the orientation of the robot base
        """
        self._robot.base = self._position@self._transform
    
    def end_factor_position(self):
        """
        give back the end-factor position
        """
        # return np.asarray(self._robot.q, dtype=float)
        return self._robot.fkine(self._robot.q).t
        
    def apply_velocity_cmd(self, qdot: np.ndarray, cond_number: float, error):
        """
        apply the modification of the joints to the robot
        """
        self._robot.qd = qdot
        self.update_current_state(cond_number)
        self._distance = np.linalg.norm(error)
        
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

    def _get_absolute_time_window(self, idx_first_task: int, idx_last_task: int, 
                                start_time: float = None, end_time: float = None):
        """
        Helper function to determine the absolute start and end timestamps 
        based on task indices or manual time overrides.
        
        Returns:
            tuple[float, float] | None: (start_time_abs, end_time_abs) or None on error.
        """
        
        # absolute start time (start_time_abs)
        try:
            if start_time is not None:
                start_time_abs = start_time
            else:
                # use first start time of the first task
                start_time_abs = self._start_time_tasks[idx_first_task]
        except IndexError:
            print("Error: Task window indices are out of range for start times.")
            return None, None

        # absolute end time (end_time_abs)
        if end_time is not None:
            end_time_abs = end_time
        elif idx_last_task < len(self._end_time_tasks):
            # use last end time of the lasr task
            end_time_abs = self._end_time_tasks[idx_last_task]
        else:
            # if no end time provided or it is out of range -> last time data provided
            end_time_abs = self._time_data[-1] if self._time_data else start_time_abs

        return start_time_abs, end_time_abs
    
    def record_collision_event(self, timestamp: float, position: np.ndarray, distance: float, had_precedence: bool):
        """
        Records a near-collision event with timestamp and position.
        
        Args:
            timestamp: Absolute time when the collision risk occurred
            position: End-effector position at the collision event
            distance: Distance between robots at collision
            had_precedence: Whether this robot had precedence (could move)
        """
        self._collision.append({
            'timestamp': timestamp,
            'position': position.copy(),
            'distance': distance,
            'had_precedence': had_precedence
        })

    def get_collision_events(self):
        """
        Returns the list of recorded collision events.
        
        Returns:
            list: List of collision event dictionaries
        """
        return self._collision
    

    def get_trajectory(self):
        """
        Retrieves the history of end-effector positions and corresponding time data.

        Returns:
            tuple[np.ndarray, np.ndarray]: (ee_positions, time_data)
        """
        ee = np.array(self._ee_pos_hist)
        t = np.array(self._time_data)

        if len(ee) == 0:
            print("No trajectory data available to plot.")
        
        return ee, t
    
    def plot_side_view(self, ee=None, ax=None, line_color="blue", f=14):
        """
        Plot on the axes or new figure the Side View's trajectory of the robot. 
        """
        if ax is None:
            fig, ax = plt.subplots(1,1,figsize=(6,5))
            show_plot = True
        else:
            show_plot = False
        
        if ee is None:
            ee, _ = self.get_trajectory()

        ax.plot(ee[:,0], ee[:,2], linewidth=1.5, color=line_color, label=self.name)
        ax.axhline(0, linestyle='--', color='brown', alpha=0.6, label="Base Level")
        ax.set_title("Side View (XZ Plane)", fontsize=f)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Z [m]")
        ax.grid(True, alpha=0.3)

        if show_plot:
            plt.tight_layout()
            plt.show()

            return fig

    def plot_front_view(self, ee=None, ax=None, line_color="blue", f=14):
        """
        Plot on the axes or new figure the Front View's trajectory of the robot. 
        """
        if ax is None:
            fig, ax = plt.subplots(1,1,figsize=(6,5))
            show_plot = True
        else:
            show_plot = False
        
        if ee is None:
            ee, _ = self.get_trajectory()

        ax.plot(ee[:,1], ee[:,2], linewidth=1.5, color=line_color, label=self.name)
        ax.axhline(0, linestyle='--', color='brown', alpha=0.6, label="Base Level")
        ax.set_title("Front View (YZ Plane)", fontsize=f)
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("Z [m]")
        ax.grid(True, alpha=0.3)

        if show_plot:
            plt.tight_layout()
            plt.show()

            return fig
        
    def plot_top_view(self, ee=None, ax=None, line_color="blue", f=14):
        """
        Plot on the axes or new figure the Top View's trajectory of the robot. 
        """
        if ax is None:
            fig, ax = plt.subplots(1,1,figsize=(6,5))
            show_plot = True
        else:
            show_plot = False
        
        if ee is None:
            ee, _ = self.get_trajectory()

        ax.plot(ee[:,0], ee[:,1], linewidth=1.5, color=line_color, label=self.name)
        # ax.scatter(ee[0,0], ee[0,1], c='green', s=120, edgecolor='black', label='Pick')
        # ax.scatter(ee[-1,0], ee[-1,1], c='red', s=120, edgecolor='black', label='Place')
        ax.axhline(0, linestyle='--', color='brown', alpha=0.6)
        ax.set_title("Top View (XY Plane)", fontsize=f)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.grid(True, alpha=0.3)

        if show_plot:
            plt.tight_layout()
            plt.show()

            return fig

    def plot_3d_trajectory_views(self, axes=None, line_color="blue", global_limits=None, f=14):
        """
        Plots the robot's end-effector trajectory in three orthogonal 2D views 
        (Top, Side, Front) on a single figure, applying global limits for uniformity.
        
        Args:
            global_limits (list, optional): axes limits [[x_min, x_max], [y_min, y_max], [z_min, z_max]].
        """
        ee, _ = self.get_trajectory()

        if axes is None:
            fig, axes = plt.subplots(1, 3, figsize=(10, 4))
            show_plot = True
        else:
            show_plot = False

        # Common settings
        start_pos = ee[0]
        end_pos = ee[-1]
        
        # 1. Top View (XY)
        self.plot_top_view(ee, axes[0], line_color, f=f)
        # axes[0].scatter(start_pos[0], start_pos[1], c='green', s=120, edgecolor='black', label='Start/Pick')
        # axes[0].scatter(end_pos[0], end_pos[1], c='red', s=120, edgecolor='black', label='End/Place')
        
        # 2. Side View (XZ Plane)
        self.plot_side_view(ee, axes[1], line_color, f=f)
        
        # 3. Front View (YZ Plane)
        self.plot_front_view(ee, axes[2], line_color, f=f)
        axes[2].legend()

        # Apply global limits
        if global_limits is None:
            axes[0].axis("equal")
            axes[1].axis("equal")
            axes[2].axis("equal")
            print("Warning: Using default limits based on current robot data only.")
        else:
            x_lims, y_lims, z_lims = global_limits
            axes[0].set_xlim(x_lims)
            axes[0].set_ylim(y_lims)
            axes[1].set_xlim(x_lims)
            axes[1].set_ylim(z_lims)
            axes[2].set_xlim(y_lims)
            axes[2].set_ylim(z_lims)
        
        # Apply common styling
        for ax in axes:
            ax.grid(True, alpha=0.3)

        if show_plot:
            fig.suptitle('End-Effector Trajectory in 3D Views', fontsize=12)
            plt.tight_layout(rect=[0, 0, 1, 0.95]) # Adjust layout to fit suptitle
            plt.show()

            return fig

    def plot_performance_metrics(self, window_tasks=None, axes=None,
                                 qd_color='k', cond_color='b',
                                 start_time=None, end_time=None):
        """
        Plots the robot's control metrics: Joint Velocities (q_dot) and 
        Jacobian Condition Number, filtered by a specified task window.
        
        Args:
            window_tasks (list, optional): [start_task_index, end_task_index] to filter data.
            axes (list[matplotlib.axes.Axes], optional): Two axes objects [ax_qd, ax_cond] to plot onto. 
                                                        If None, a new figure is created.
            qd_color (str, optional): Color for Joint Velocities (q_dot) line.
            cond_color (str, optional): Color for Jacobian Condition Number (kappa) line.
            start_time (float, optional): Absolute time override for plot start.
            end_time (float, optional): Absolute time override for plot end.
        """
        # retrieve relatives times
        if window_tasks is None:
            window_tasks = [0, len(self._start_time_tasks)]
        
        idx_first_task = window_tasks[0]
        idx_last_task = window_tasks[-1]

        if idx_last_task >= len(self._start_time_tasks):
            idx_last_task = len(self._start_time_tasks) - 1

        start_time_abs, end_time_abs = self._get_absolute_time_window(idx_first_task, idx_last_task,
                                                                      start_time, end_time)
        
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
        
        # plotting
        if axes is None:
            fig, axes = plt.subplots(2, 1, figsize=(10, 4))
            show_plot = True
        else:
            show_plot = False
        
        ax_qd, ax_cond = axes

        # 1. Plot Joint Velocities (q_dot)
        ax_qd.plot(time_data_plot, qd_hist_plot, color=qd_color, label=f'{self.name}_ Joint Velocity ($\\dot{{q}}$)')

        # 2. Plot Condition Number (kappa)
        ax_cond.plot(time_data_plot, cond_hist_plot, color=cond_color, linestyle='--',
                     label=f'{self.name}_ Jacobian Condition Number ($\\kappa$)')
        
        # 3. add markers

        for ax in axes:
            ax.set_xlim(0, time_data_plot[-1])
            ax.grid(True)

        has_label = False
        for t in task_markers:
            label = f"{self.name} _ Task End Boundary" if not has_label else None
            ax_qd.axvline(x=t, linestyle='--', color=cond_color, alpha=0.4, label=label)
            ax_cond.axvline(x=t, linestyle='--', color=qd_color, alpha=0.4, label=label)

            has_label = True

        ax_qd.set_title(f'{self.name} Control Performance Metrics')
        #.title('Robot Control Performance Metrics - joint positions')
        ax_qd.set_ylabel(r'$q_i [rad]$')
        ax_qd.set_xlim(0, time_data_plot[-1]) 
        ax_qd.grid(True, alpha=0.5, color='0.95')
        # ax_qd.legend(loc='best')
        
        ax_cond.set_xlabel('Time [s]')
        ax_cond.set_ylabel(r'Condition number ($\kappa$)')
        ax_cond.set_xlim(0, time_data_plot[-1]) 
        ax_cond.grid(True, alpha=0.5, color='0.95')
        ax_cond.legend(loc='best')
        
        if show_plot:
            plt.tight_layout()
            plt.show()

            return fig

    def plot_height_over_time(self, window_tasks=None, start_time=None, end_time=None,
                              ax=None, line_color='purple', global_limits=None, f=14):
        """
        Plots the end-effector's vertical position (Z-coordinate) as a function of time,
        using relative time and showing task separation markers, filtered by task window.
        
        Args:
            window_tasks (list, optional): [start_task_index, end_task_index] to filter data.
            start_time (float, optional): Absolute time override for plot start.
            end_time (float, optional): Absolute time override for plot end.
            ax (matplotlib.axes.Axes, optional): Specific axes object to plot onto. 
                                                If None, a new figure is created.
            line_color (str, optional): Color of the height trajectory line.
            global_z_limits (list, optional): [min_z, max_z] limits for the Y-axis (Z position).
        """
        # retrieve relatives times
        if window_tasks is None:
            window_tasks = [0, len(self._start_time_tasks)]
        
        idx_first_task = window_tasks[0]
        idx_last_task = window_tasks[-1]

        if idx_last_task >= len(self._start_time_tasks):
            idx_last_task = len(self._start_time_tasks) - 1

        start_time_abs, end_time_abs = self._get_absolute_time_window(idx_first_task, idx_last_task,
                                                                      start_time, end_time)
        if start_time is not None:
            start_time_abs = start_time
        if end_time is not None:
            end_time_abs = end_time

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

        # prepare task vertical lines (scaled to relative time)
        # only in the window
        task_end_markers = [
            t - start_time_abs for t in self._end_time_tasks 
            if start_time_abs <= t <= end_time_abs
        ]
        
        # plotting
        if ax is None:
            fig, ax = plt.subplots(1, 1, figsize=(10, 5))
            #plt.figure('Height Over Time', figsize=(10, 5))
            show_plot = True
        else:
            show_plot = False
        
        # height wrt Z
        ax.plot(time_data_plot, ee_z_plot, linewidth=1.5, color=line_color, label=f'{self.name} Height')
        ax.axhline(0, linestyle='--', color='brown', alpha=0.6, label="Table Level" if ax.get_legend() is None else None)

        # Adds vertical lines as tasks' separators
        # flag to add label once in the legend
        has_label = False
        for t in task_end_markers:
            label = "Task End Boundary" if not has_label else None
            ax.axvline(t, linestyle='--', color='red', alpha=0.4, label=label)
            has_label = True

        if ax.get_title() == '':
            ax.set_title("End-Effector Height (Z) Over Relative Time", fontsize=f)
            ax.set_xlabel("Time [s] (Relative to Task Start)")
            ax.set_ylabel("Z Position [m]")

        # apply limits referred to z axis
        if global_limits is not None:
            ax.set_ylim(global_limits[2])

        ax.set_xlim(0, time_data_plot[-1]) 
        
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')

        if show_plot:
            plt.tight_layout()
            plt.show()

            return fig

    def plot_collision_markers_3d(self, axes, start_time=None, end_time=None, 
                               marker_color='red', marker_size=100):
        """
        Adds collision event markers to existing 3D trajectory views.
        
        Args:
            axes: List of 3 axes [top_view, side_view, front_view]
            start_time: Filter events after this time
            end_time: Filter events before this time
            marker_color: Color for collision markers
            marker_size: Size of the markers
        """
        if len(self._collision) == 0:
            return
        
        # Filter events by time window
        filtered_events = self._collision
        if start_time is not None or end_time is not None:
            filtered_events = [
                e for e in self._collision
                if (start_time is None or e['timestamp'] >= start_time) and
                (end_time is None or e['timestamp'] <= end_time)
            ]
        
        if len(filtered_events) == 0:
            return
        
        # Extract positions
        positions = np.array([e['position'] for e in filtered_events])
        
        # Plot on each view
        ax_top, ax_side, ax_front = axes
        
        # Top view (XY)
        ax_top.scatter(positions[:, 0], positions[:, 1], 
                    c=marker_color, s=marker_size, marker='x', 
                    alpha=0.7, linewidths=2, zorder=10,
                    label=f'{self.name} Near-Collision')
        
        # Side view (XZ)
        ax_side.scatter(positions[:, 0], positions[:, 2], 
                        c=marker_color, s=marker_size, marker='x', 
                        alpha=0.7, linewidths=2, zorder=10)
        
        # Front view (YZ)
        ax_front.scatter(positions[:, 1], positions[:, 2], 
                        c=marker_color, s=marker_size, marker='x', 
                        alpha=0.7, linewidths=2, zorder=10)

    def plot_collision_markers_time(self, ax, start_time=None, end_time=None,
                                    marker_color='red', alpha=0.3):
        """
        Adds vertical lines at collision events on time-based plots.
        
        Args:
            ax: Matplotlib axis to add markers to
            start_time: Reference start time for relative plotting
            end_time: Filter events before this time
            marker_color: Color for the vertical lines
            alpha: Transparency of the lines
        """
        if len(self._collision) == 0:
            return
        
        # Use first collision event time as reference if start_time not provided
        if start_time is None:
            start_time = self._collision[0]['timestamp'] if self._collision else 0
        
        # Filter and convert to relative time
        relative_times = []
        for e in self._collision:
            if end_time is None or e['timestamp'] <= end_time:
                relative_times.append(e['timestamp'] - start_time)
        
        # Plot vertical lines
        has_label = False
        for t in relative_times:
            if t >= 0:  # Only plot if within the time window
                label = f'{self.name} Near-Collision' if not has_label else None
                ax.axvline(t, linestyle=':', color=marker_color, alpha=alpha, 
                        linewidth=2, label=label)
                has_label = True