from classes.robot import *
from classes.controller import *
from classes.objects import *
from classes.task_manager import *
from typing import Tuple
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

import time
import os

GAIN = 2.5
DT = 0.01

PLOTS_DIR = './plots'


# general function computations
def save_image(fig, file_name, dir_path=PLOTS_DIR):
    """
    Save the image in the given path, with the given name
    
    :param fig: Figure created with matplotlib
    :param file_name: name to save the figure (e.g. 'image.png')
    :param dir_path: directory's path in which save the image. Defaults './plots'. 
    """
    file_path = os.path.join(dir_path, file_name)

    if not os.path.exists(dir_path):
        os.makedirs(dir_path)
        print(f"Created directory: {PLOTS_DIR}")

    try:
        fig.savefig(file_path, bbox_inches='tight', dpi=300)
        print(f"Image successfully saved to: {file_path}")
    except Exception as e:
        print(f"Error saving figure: {e}")

def global_limits(robots, n_dims=3, marg=0.2):
    xyz_min = [np.inf] * 3
    xyz_max = [-np.inf] * 3

    for robot in robots:
        ee, _ = robot.get_trajectory()
        # skip robot with no data
        if len(ee) == 0:
            continue
        for i in range(n_dims): # i = 0 (X), 1 (Y), 2 (Z)
            # find min and max on the current dimension i
            i_min = np.min(ee[:,i]) - marg
            i_max = np.max(ee[:,i]) + marg
            # update local variable
            xyz_min[i] = min(xyz_min[i], i_min)
            xyz_max[i] = max(xyz_max[i], i_max)

    # since
    # xyz_min = x_min, y_min, z_min
    # xyz_max = x_max, y_max, z_max
    # global_limits = [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
    global_limits = [[xyz_min[dim],xyz_max[dim]] for dim in range(n_dims)]

    # if min and max coincides
    for dim in range(n_dims):
        if global_limits[dim][0] == global_limits[dim][1]:
            center = global_limits[dim][0]
            global_limits[dim] = [center - 0.1, center + 0.1]
            
    return global_limits

# subplots asking to the robot class to compile it
def plot_all_metrics_combined(panda_agent_1, panda_agent_2, global_limits_3d=None, start_time=None, end_time=None, figsize=((20,6), (15, 5), (15, 5)), show=True, f=8):
    """
    Create an unique figure with all metrics and trajectories
    """
    fig_1 = plt.figure(figsize=figsize[0])
    fig_2 = plt.figure(figsize=figsize[1]) 
    fig_3 = plt.figure(figsize=figsize[2]) 
    
    # Define the grid 
    gs_1 = GridSpec(2, 2, figure=fig_1, hspace=0.17, wspace=0.2)
    gs_2 = GridSpec(2, 1, figure=fig_2, hspace=0.2, wspace=0.2)
    gs_3 = GridSpec(1, 3, figure=fig_3, hspace=0.2, wspace=0.2)

    # Metrics (qd and cond) 
    # for robot 1
    ax_qd_1   = fig_1.add_subplot(gs_1[0, 0])  # Robot 1: q_dot
    ax_cond_1 = fig_1.add_subplot(gs_1[1, 0], sharex=ax_qd_1) # Robot 1: K
    
    # for robot 2
    ax_qd_2   = fig_1.add_subplot(gs_1[0, 1], sharey=ax_qd_1)  # Robot 2: q_dot
    ax_cond_2 = fig_1.add_subplot(gs_1[1, 1], sharex=ax_qd_2) # Robot 2: K
    
    # Height across time
    ax_height = fig_2.add_subplot(gs_2[0, 0])
    # Inter-robot distance
    ax_distance = fig_2.add_subplot(gs_2[1, 0], sharex=ax_height)

    # 3d trajectory (Views XY, XZ, YZ) 
    ax_xy_traj = fig_3.add_subplot(gs_3[0, 0])
    ax_xz_traj = fig_3.add_subplot(gs_3[0, 1])
    ax_yz_traj = fig_3.add_subplot(gs_3[0, 2])
    
    # Robot 1: Plot metrics on axes [ax_qd_1, ax_cond_1]
    panda_agent_1.plot_performance_metrics(axes=[ax_qd_1, ax_cond_1], qd_color='blue', cond_color='darkblue')
    # Robot 2: Plot metrics on axes [ax_qd_2, ax_cond_2]
    panda_agent_2.plot_performance_metrics(axes=[ax_qd_2, ax_cond_2], qd_color='orange', cond_color='darkorange')

    # 3D Trajectory
    axes_traj_3d = [ax_xy_traj, ax_xz_traj, ax_yz_traj]
    panda_agent_1.plot_3d_trajectory_views(axes=axes_traj_3d, line_color='blue', f=f)# , global_limits=global_limits_3d
    panda_agent_2.plot_3d_trajectory_views(axes=axes_traj_3d, line_color='orange', f=f) # , global_limits=global_limits_3d

    # Height across time
    panda_agent_1.plot_height_over_time(ax=ax_height, line_color='blue', 
                                        start_time=start_time, end_time=end_time, 
                                        global_limits=global_limits_3d, f=f)
    panda_agent_2.plot_height_over_time(ax=ax_height, line_color='orange', 
                                        start_time=start_time, end_time=end_time, 
                                        global_limits=global_limits_3d, f=f)
    
    # Collision markers on time-based plots
    for ax in [ax_qd_1, ax_cond_1, ax_height]:
        panda_agent_1.plot_collision_markers_time(ax, start_time=start_time, 
                                                  end_time=end_time, marker_color='red')
    
    for ax in [ax_qd_2, ax_cond_2, ax_height]:
        panda_agent_2.plot_collision_markers_time(ax, start_time=start_time, 
                                                  end_time=end_time, marker_color='red')

    # Plot inter-robot distance over time
    plot_inter_robot_distance(panda_agent_1, panda_agent_2, ax_distance, 
                              start_time=start_time, end_time=end_time)

    ax_qd_1.set_xlabel('') 
    ax_qd_2.set_xlabel('') 
    ax_height.set_xlabel('')
    ax_distance.set_xlabel('Time [s]')
    
    for ax in axes_traj_3d:
        ax.axis("equal")
        #ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='lower right', fontsize=8)
    
    axes = [ax_cond_1, ax_cond_2, ax_height, ax_distance]
    for ax in axes:
        ax.legend(loc='lower right', fontsize=8)

    ax_cond_1.legend([])
    ax_cond_2.legend([])
    
    fig_1.suptitle('Collaborative Robotics: Performance Analysis', fontsize=18)
    fig_2.suptitle('Collaborative Robotics: Collision Analysis', fontsize=18)
    fig_3.suptitle('Collaborative Robotics: Trajectory Analysis', fontsize=18)
    
    if show:
        plt.show()

    return fig_1, fig_2, fig_3

def plot_inter_robot_distance(robot1, robot2, ax=None, start_time=None, end_time=None, f=8):
    """
    Plot the distance between two robots' end-effectors over time.
    """
    standalone = (ax is None)
    
    if standalone:
        fig, ax = plt.subplots(figsize=(12, 4))
    else:
        fig = ax.get_figure()

    # Get trajectories
    ee1, t1 = robot1.get_trajectory()
    ee2, t2 = robot2.get_trajectory()
    
    # Use the common time base (should be synchronized)
    # Find common time range
    if start_time is None:
        start_time = max(t1[0], t2[0])
    if end_time is None:
        end_time = min(t1[-1], t2[-1])
    
    # Filter data
    mask1 = (t1 >= start_time) & (t1 <= end_time)
    mask2 = (t2 >= start_time) & (t2 <= end_time)
    
    t1_filtered = t1[mask1] - start_time
    t2_filtered = t2[mask2] - start_time
    ee1_filtered = ee1[mask1]
    ee2_filtered = ee2[mask2]
    
    # Interpolate to common time base (use the finer one)
    if len(t1_filtered) > len(t2_filtered):
        t_common = t1_filtered
        # Interpolate ee2
        ee2_interp = np.array([
            np.interp(t_common, t2_filtered, ee2_filtered[:, i]) 
            for i in range(3)
        ]).T
        ee1_interp = ee1_filtered
    else:
        t_common = t2_filtered
        # Interpolate ee1
        ee1_interp = np.array([
            np.interp(t_common, t1_filtered, ee1_filtered[:, i]) 
            for i in range(3)
        ]).T
        ee2_interp = ee2_filtered
    
    # Calculate distances
    distances = np.linalg.norm(ee1_interp - ee2_interp, axis=1)
    
    # Plot
    ax.plot(t_common, distances, linewidth=1.5, color='purple', label='Inter-Robot Distance', alpha=0.9)
    ax.axhline(0.2, linestyle='--', linewidth=1.5, color='red', alpha=0.7,
               label='Safety Threshold (0.2m)')
    
    # Highlight regions below threshold
    below_threshold = distances < 0.2
    if np.any(below_threshold):
        ax.fill_between(t_common, 0, distances, where=below_threshold, 
                        alpha=0.3, color='red', label='Collision Risk Zone')
    
    ax.set_ylabel('Distance [m]')
    ax.set_title('Inter-Robot Distance Over Time', fontsize=f)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.set_xlim(0, t_common[-1])

    if standalone:
        plt.show()
    
    return fig

# TASK INITIALIZATIONS
def initialize_task_1a() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 1a: one panda robot that build one single tower
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    robots = [panda_agent]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15))

    brick_B1 = Brick(sm.SE3(0.11, 0.5, 0.05))
    brick_B2 = Brick(sm.SE3(0.11, 0.5, 0.15))

    brick_C1 = Brick(sm.SE3(0.22, 0.5, 0.05))
    brick_C2 = Brick(sm.SE3(0.22, 0.5, 0.15))

    brick_D1 = Brick(sm.SE3(-0.11, 0.5, 0.05))
    brick_D2 = Brick(sm.SE3(-0.11, 0.5, 0.15))

    bricks = [brick_A1, brick_A2, brick_B1, brick_B2,
              brick_C1, brick_C2, brick_D1, brick_D2]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=6)
    
    towers = [tower_A]

    return robots, bricks, towers

def initialize_task_1b() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 1b: two panda robots that build one single tower
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    panda_agent_2 = Robot_arm("panda_2", sm.SE3.Rz(np.pi), sm.SE3.Tx(0.2))
    panda_agent_2.set_position(0.70, 0.0, 0.0)
    panda_agent_2.modify_orientation_base()

    robots = [panda_agent, panda_agent_2]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.05))

    brick_B1 = Brick(sm.SE3(0.11, 0.5, 0.05))
    brick_B2 = Brick(sm.SE3(0.11, 0.5, 0.15))

    brick_C1 = Brick(sm.SE3(0.22, 0.5, 0.05))
    brick_C2 = Brick(sm.SE3(0.22, 0.5, 0.15))

    brick_D1 = Brick(sm.SE3(-0.11, 0.5, 0.05))
    brick_D2 = Brick(sm.SE3(-0.11, 0.5, 0.15))

    bricks = [brick_A1, brick_A2, brick_B1, brick_B2,
              brick_C1, brick_C2, brick_D1, brick_D2]
    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=6)
    
    towers = [tower_A]

    return robots, bricks, towers

def initialize_task_2a() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 2a: one panda robot that build a wall
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    robots = [panda_agent]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_A3 = Brick(sm.SE3(0.0, 0.5, 0.25))
    brick_A4 = Brick(sm.SE3(0.0, 0.5, 0.35))

    brick_B1 = Brick(sm.SE3(-0.11, 0.5, 0.05))
    brick_B2 = Brick(sm.SE3(-0.11, 0.5, 0.15))
    brick_B3 = Brick(sm.SE3(-0.11, 0.5, 0.25))
    brick_B4 = Brick(sm.SE3(-0.11, 0.5, 0.35))

    brick_C1 = Brick(sm.SE3(-0.22, 0.5, 0.05))
    brick_C2 = Brick(sm.SE3(-0.22, 0.5, 0.15))
    brick_C3 = Brick(sm.SE3(-0.22, 0.5, 0.25))
    brick_C4 = Brick(sm.SE3(-0.22, 0.5, 0.35))

    brick_D1 = Brick(sm.SE3(0.11, 0.5, 0.05))
    brick_D2 = Brick(sm.SE3(0.11, 0.5, 0.15))
    

    bricks = [brick_A1, brick_A2, brick_A3, brick_A4,
              brick_B1, brick_B2, brick_B3, brick_B4,
              brick_C1, brick_C2, brick_C3, brick_C4,
              brick_D1, brick_D2]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=4)
    
    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=4)
    
    base_C = sm.SE3(0.0, 0.0, 0.05)
    tower_C = Tower(base_pose=base_C, max_height=4)
    
    towers = [tower_A, tower_B, tower_C]

    return robots, bricks, towers

def initialize_task_2b() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 2b: two panda robot that build a wall
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    panda_agent_2 = Robot_arm("panda_2", sm.SE3.Rz(np.pi), sm.SE3.Tx(0.2))
    panda_agent_2.set_position(0.70, 0.0, 0.0)
    panda_agent_2.modify_orientation_base()

    robots = [panda_agent, panda_agent_2]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_A3 = Brick(sm.SE3(0.0, 0.5, 0.25))

    brick_B1 = Brick(sm.SE3(0.11, 0.5, 0.05))
    brick_B2 = Brick(sm.SE3(0.11, 0.5, 0.15))
    brick_B3 = Brick(sm.SE3(0.11, 0.5, 0.25))

    brick_C1 = Brick(sm.SE3(0.22, 0.5, 0.05))
    brick_C2 = Brick(sm.SE3(0.22, 0.5, 0.15))
    brick_C3 = Brick(sm.SE3(0.22, 0.5, 0.25))

    brick_D1 = Brick(sm.SE3(-0.11, 0.5, 0.05))
    brick_D2 = Brick(sm.SE3(-0.11, 0.5, 0.15))
    brick_D3 = Brick(sm.SE3(-0.11, 0.5, 0.25))

    bricks = [brick_A1, brick_A2, brick_A3, brick_B1, brick_B2, brick_B3,
              brick_C1, brick_C2, brick_C3, brick_D1, brick_D2, brick_D3]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=4)
    
    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=4)
    
    base_C = sm.SE3(0.0, 0.0, 0.05)
    tower_C = Tower(base_pose=base_C, max_height=4)
    
    towers = [tower_A, tower_B, tower_C]

    return robots, bricks, towers

def initialize_task_3a() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 3a: one panda robot that build two towers following a pattern
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    robots = [panda_agent]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05), color_name="blue")
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15), color_name="green")
    brick_A3 = Brick(sm.SE3(0.0, 0.5, 0.25), color_name="blue")
    brick_A4 = Brick(sm.SE3(0.0, 0.5, 0.35), color_name="green")
    brick_A5 = Brick(sm.SE3(0.0, 0.5, 0.45), color_name="blue")

    brick_B1 = Brick(sm.SE3(-0.11, 0.5, 0.05), color_name="green")
    brick_B2 = Brick(sm.SE3(-0.11, 0.5, 0.15), color_name="blue")
    brick_B3 = Brick(sm.SE3(-0.11, 0.5, 0.25), color_name="green")
    brick_B4 = Brick(sm.SE3(-0.11, 0.5, 0.35), color_name="blue")
    brick_B5 = Brick(sm.SE3(-0.11, 0.5, 0.45), color_name="green")

    bricks = [brick_A1, brick_A2, brick_A3, brick_A4, brick_A5,
              brick_B1, brick_B2, brick_B3, brick_B4, brick_B5]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=4, color_name = "Blue")

    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=4, color_name = "Green")

    towers = [tower_A, tower_B]

    return robots, bricks, towers

def initialize_task_3b() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 3b: two panda robot that build two towers following a pattern
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    panda_agent_2 = Robot_arm("panda_2", sm.SE3.Rz(np.pi), sm.SE3.Tx(0.2))
    panda_agent_2.set_position(0.70, 0.0, 0.0)
    panda_agent_2.modify_orientation_base()

    robots = [panda_agent, panda_agent_2]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05), color_name="blue")
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15), color_name="green")

    brick_B1 = Brick(sm.SE3(0.11, 0.5, 0.05), color_name="green")
    brick_B2 = Brick(sm.SE3(0.11, 0.5, 0.15), color_name="blue")

    brick_C1 = Brick(sm.SE3(0.22, 0.5, 0.05), color_name="blue")
    brick_C2 = Brick(sm.SE3(0.22, 0.5, 0.15), color_name="green")

    brick_D1 = Brick(sm.SE3(-0.11, 0.5, 0.05), color_name="green")
    brick_D2 = Brick(sm.SE3(-0.11, 0.5, 0.15), color_name="blue")

    bricks = [brick_A1, brick_A2, brick_B1, brick_B2,
              brick_C1, brick_C2, brick_D1, brick_D2]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=4, color_name = "Blue")

    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=4, color_name = "Green")

    towers = [tower_A, tower_B]

    return robots, bricks, towers

def initialize_task_4a() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 4a: one panda robot that build a wall following a pattern
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    robots = [panda_agent]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05), color_name="green")
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15), color_name="blue")
    brick_A3 = Brick(sm.SE3(0.0, 0.5, 0.25), color_name="red")
    brick_A4 = Brick(sm.SE3(0.0, 0.5, 0.35), color_name="green")

    brick_B1 = Brick(sm.SE3(-0.11, 0.5, 0.05), color_name="green")
    brick_B2 = Brick(sm.SE3(-0.11, 0.5, 0.15), color_name="red")
    brick_B3 = Brick(sm.SE3(-0.11, 0.5, 0.25), color_name="blue")
    brick_B4 = Brick(sm.SE3(-0.11, 0.5, 0.35), color_name="red")

    brick_C1 = Brick(sm.SE3(-0.22, 0.5, 0.05), color_name="red")
    brick_C2 = Brick(sm.SE3(-0.22, 0.5, 0.15), color_name="blue")
    brick_C3 = Brick(sm.SE3(-0.22, 0.5, 0.25), color_name="green")
    brick_C4 = Brick(sm.SE3(-0.22, 0.5, 0.35), color_name="blue")

    bricks = [brick_A1, brick_A2, brick_A3, brick_A4,
              brick_B1, brick_B2, brick_B3, brick_B4,
              brick_C1, brick_C2, brick_C3, brick_C4]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=4, color_name = "Blue")

    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=4, color_name = "Green")

    base_C = sm.SE3(0.0, 0.0, 0.05)
    tower_C = Tower(base_pose=base_C, max_height=4, color_name = "Red")

    towers = [tower_A, tower_B, tower_C]

    return robots, bricks, towers

def initialize_task_4b() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 4b: two panda robot that build a wall following a pattern
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    panda_agent_2 = Robot_arm("panda_2", sm.SE3.Rz(np.pi), sm.SE3.Tx(0.2))
    panda_agent_2.set_position(0.70, 0.0, 0.0)
    panda_agent_2.modify_orientation_base()

    robots = [panda_agent, panda_agent_2]

    # bricks
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.05), color_name="green")
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15), color_name="blue")
    brick_A3 = Brick(sm.SE3(0.0, 0.5, 0.25), color_name="red")

    brick_B1 = Brick(sm.SE3(0.11, 0.5, 0.05), color_name="green")
    brick_B2 = Brick(sm.SE3(0.11, 0.5, 0.15), color_name="red")
    brick_B3 = Brick(sm.SE3(0.11, 0.5, 0.25), color_name="blue")

    brick_C1 = Brick(sm.SE3(0.22, 0.5, 0.05), color_name="red")
    brick_C2 = Brick(sm.SE3(0.22, 0.5, 0.15), color_name="blue")
    brick_C3 = Brick(sm.SE3(0.22, 0.5, 0.25), color_name="green")

    brick_D1 = Brick(sm.SE3(-0.11, 0.5, 0.05), color_name="green")
    brick_D2 = Brick(sm.SE3(-0.11, 0.5, 0.15), color_name="red")
    brick_D3 = Brick(sm.SE3(-0.11, 0.5, 0.25), color_name="blue")

    bricks = [brick_A1, brick_A2, brick_A3, brick_B1, brick_B2, brick_B3,
              brick_C1, brick_C2, brick_C3, brick_D1, brick_D2, brick_D3]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=4, color_name = "Blue")

    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=4, color_name = "Green")

    base_C = sm.SE3(0.0, 0.0, 0.05)
    tower_C = Tower(base_pose=base_C, max_height=4, color_name = "Red")

    towers = [tower_A, tower_B, tower_C]

    return robots, bricks, towers

# TASK DEFINITION AND RUNNING
def run_task(env: swift.Swift, robots: List[Robot_arm], bricks: List[Brick], towers: List[Tower]) -> Tuple[int]:
    """
    Run the general task given the resources
    """
    # add elements to the environment
    for r in robots:
        r.register(env)

    for b in bricks:
        env.add(b.obj)

    # initialize the sensor
    sensor = Sensor(env, bricks=bricks, towers=towers, robots=robots)
    # initialize the controller of the robot  
    controller = Controller(env, gain=GAIN) 
    # initialize the task manager
    task_manager = TaskManager(env, sensor, robots=robots, controller=controller, dt=DT)

    start_time = time.time()
    task_manager.start()
    end_time=time.time()

    return start_time, end_time

def task(name: str, all_plots: bool = False, global_plot: bool = True, save: bool = False, dir_path: str = PLOTS_DIR):
    """
    Initialize, perform and plot the metrics of a given task:
    - task '1a': one panda robot that build one single tower
    - task '2a': one panda robot that build a wall
    - task '3a': one panda robot that build two towers following a pattern
    - task '4a': one panda robot that build a wall following a pattern
    - task '1b': two panda robot that build one single tower
    - task '2b': two panda robot that build a wall
    - task '3b': two panda robot that build two towers following a pattern
    - task '4b': two panda robot that build a wall following a pattern

    Args:
        name (str): index of the task
        all_plots (bool, optional): whether show all singular plots [qdot, condition number, height over time, views plots]. Defaults False.
        global_plots (bool, optional): whether show complex and coupled plots [summary of the task, collision plot]. Defaults True.
        save (bool, optional): whether to save in the correct directory the plots created. Deafults False.
        dir_path (str, optional): where is the main directory in the Disk. Defaults to './plots'.
    """
    # create the swift enviroment
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    
    print('===========================================================================')
    print(f'\t\tTASK {name}: ', end='')
    if name == '1a':
        print('ONE PANDA BUILD ONE TOWER')
        robots, bricks, towers = initialize_task_1a()
    elif name == '1b':
        print('TWO PANDA BUILD ONE TOWER')
        robots, bricks, towers = initialize_task_1b()
    elif name == '2a':
        print('ONE PANDA BUILD WALL')
        robots, bricks, towers = initialize_task_2a()
    elif name == '2b':
        print('TWO PANDA BUILD WALL')
        robots, bricks, towers = initialize_task_2b()
    elif name == '3a':
        print('ONE PANDA BUILD TWO TOWER WITH DEFINED COLORS')
        robots, bricks, towers = initialize_task_3a()
    elif name == '3b':
        print('ONE PANDA BUILD TWO TOWER WITH DEFINED COLORS')
        robots, bricks, towers = initialize_task_3b()
    elif name == '4a':
        print('ONE PANDA BUILD WALL WITH DEFINED COLORS')
        robots, bricks, towers = initialize_task_4a()
    elif name == '4b':
        print('TWO PANDA BUILD WALL WITH DEFINED COLORS')
        robots, bricks, towers = initialize_task_4b()
    elif name == 'exit': # wanna exit the loop
        return 0
    else:
        print('|  !!!!!!!!            Invalid input ...            !!!!!!!!  |')
        # task(name, all_plots, global_plot, save, dir_path)
        return None
    print('===========================================================================')

    start_time, end_time = run_task(env, robots, bricks, towers)

    print(f'global time: {end_time-start_time} seconds')

    global_range = global_limits(robots)

    task_directory = os.path.join(dir_path, f'_{name}')

    
    if global_plot and len(robots) > 1:
        glob_fig_1, glob_fig_2, glob_fig_3 = plot_all_metrics_combined(robots[0], robots[1], global_range, start_time=start_time, end_time=end_time)
        coll_fig = plot_inter_robot_distance(robots[0], robots[1], start_time=start_time, end_time=end_time)
        if save:
            save_image(glob_fig_1, 'plot_performances.png', task_directory)
            save_image(glob_fig_2, 'plot_collisions.png', task_directory)
            save_image(glob_fig_3, 'plot_trajectories.png', task_directory)
            save_image(coll_fig, 'plot_inter_robot_distance.png', task_directory)
    
    if all_plots or (global_plot and len(robots)==1):
        colors = ['blue', 'orange']
        for i in range(len(robots)):
            r = robots[i]
            views_fig = r.plot_3d_trajectory_views(line_color=colors[i])# global_limits=global_range,
            heigth_fig = r.plot_height_over_time(start_time=start_time, end_time=end_time, global_limits=global_range, line_color=colors[i])
            joint_fig = r.plot_performance_metrics(start_time=start_time, end_time=end_time, qd_color='b', cond_color=colors[i])

            if save:
                save_image(views_fig, f'3d_trajectory_views[{r.name}].png', task_directory)
                save_image(heigth_fig, f'plot_height_over_time[{r.name}].png', task_directory)
                save_image(joint_fig, f'plot_joints_metrics[{r.name}].png', task_directory)
    return 1

def retrieve_task_name() -> str:    
    name = input( "Select one of the following task: " \
                  "\n- task '1a': one panda robot that build one single tower" \
                  "\n- task '2a': one panda robot that build a wall" \
                  "\n- task '3a': one panda robot that build two towers following a pattern" \
                  "\n- task '4a': one panda robot that build a wall following a pattern" \
                  "\n- task '1b': two panda robot that build one single tower" \
                  "\n- task '2b': two panda robot that build a wall" \
                  "\n- task '3b': two panda robot that build two towers following a pattern" \
                  "\n- task '4b': two panda robot that build a wall following a pattern" \
                  "\n\n-- digit 'exit' to kill the script" \
                  "\nSELECTION (e.g.: '1a', '1b', ...):" \
                  "\n\ttask: "
                  )
    
    return name

def main():
    print(f"\n\n==> IN THE FIRST LINES OF THE CODE; \n    THERE ARE TWO VARIABLES THAT CAN BE MODIFIED, here the defaults ones:" \
          f"\n\t\t GAIN = {GAIN}\t\t bigger it is, faster the movements" \
          f"\n\t\t DT = {DT}\t\t bigger it is, bigger are the movements" \
         )
    
    task_name = retrieve_task_name()
    if task_name is None:
        main()
    elif task_name == 'exit':
        return 0
    elif task(task_name, global_plot=True, all_plots=False, save=True):  # put True True to viz all plots
        main()  # task ended successfully
    else: # task doesn't end successfully
        print("TASK NOT END AS EXPECTED")
        return

if __name__ == "__main__":
    main()