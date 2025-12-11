from classes.robot import *
from classes.controller import *
from classes.objects import *
from classes.task_manager import *

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
# from classes.sensor import *
# from classes import *

import time
import os

PLOTS_DIR = './plots'

# TASK INITIALIZATIONS
def initialize_task_1() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 1: one panda robot that build one single tower
    
    :return: three lists, one of the agents, one of the bricks in the environment and one for the towers to build
    :rtype: Tuple[List[Robot_arm], List[Brick], List[Tower]]
    """
    # robot
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()

    robots = [panda_agent]

    # bricks
    brick_A = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_B = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_C = Brick(sm.SE3(0.11, 0.5, 0.15))
    brick_D = Brick(sm.SE3(0.11, 0.5, 0.05))

    brick_E = Brick(sm.SE3(0.22, 0.5, 0.15))
    brick_F = Brick(sm.SE3(0.22, 0.5, 0.05))
    brick_G = Brick(sm.SE3(-0.11, 0.5, 0.15))
    brick_H = Brick(sm.SE3(-0.11, 0.5, 0.05))

    bricks = [brick_A, brick_B, brick_C, brick_D, brick_E, brick_F, brick_G, brick_H]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=6)
    
    towers = [tower_A]

    return robots, bricks, towers

def initialize_task_2() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 2: two panda robots that build one single tower
    
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
    brick_A = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_B = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_C = Brick(sm.SE3(0.11, 0.5, 0.15))
    brick_D = Brick(sm.SE3(0.11, 0.5, 0.05))

    brick_E = Brick(sm.SE3(0.22, 0.5, 0.15))
    brick_F = Brick(sm.SE3(0.22, 0.5, 0.05))
    brick_G = Brick(sm.SE3(-0.11, 0.5, 0.15))
    brick_H = Brick(sm.SE3(-0.11, 0.5, 0.05))

    bricks = [brick_A, brick_B, brick_C, brick_D, brick_E, brick_F, brick_G, brick_H]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=6)
    
    towers = [tower_A]

    return robots, bricks, towers

def initialize_task_3() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 3: two panda robot that build a wall
    
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
    brick_A1 = Brick(sm.SE3(0.0, 0.5, 0.25))
    brick_A2 = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_A3 = Brick(sm.SE3(0.0, 0.5, 0.05))

    brick_B1 = Brick(sm.SE3(0.11, 0.5, 0.25))
    brick_B2 = Brick(sm.SE3(0.11, 0.5, 0.15))
    brick_B3 = Brick(sm.SE3(0.11, 0.5, 0.05))

    brick_C1 = Brick(sm.SE3(0.22, 0.5, 0.25))
    brick_C2 = Brick(sm.SE3(0.22, 0.5, 0.15))
    brick_C3 = Brick(sm.SE3(0.22, 0.5, 0.05))

    brick_D1 = Brick(sm.SE3(-0.11, 0.5, 0.25))
    brick_D2 = Brick(sm.SE3(-0.11, 0.5, 0.15))
    brick_D3 = Brick(sm.SE3(-0.11, 0.5, 0.05))

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

def initialize_task_4() -> Tuple[List[Robot_arm], List[Brick], List[Tower]]:
    """
    materials occured for
    task 4: two panda robot that build a wall following a pattern
    
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
    brick_A = Brick(sm.SE3(0.0, 0.5, 0.15), color_name="Green")
    brick_B = Brick(sm.SE3(0.0, 0.5, 0.05), color_name="Red")
    brick_C = Brick(sm.SE3(0.11, 0.5, 0.15), color_name="Blue")
    brick_D = Brick(sm.SE3(0.11, 0.5, 0.05), color_name="Blue")

    brick_E = Brick(sm.SE3(0.22, 0.5, 0.15), color_name="green")
    brick_F = Brick(sm.SE3(0.22, 0.5, 0.05), color_name="green")
    brick_G = Brick(sm.SE3(-0.11, 0.5, 0.15), color_name="Red")
    brick_H = Brick(sm.SE3(-0.11, 0.5, 0.05), color_name="Blue")

    bricks = [brick_A, brick_B, brick_C, brick_D, brick_E, brick_F, brick_G, brick_H]

    # towers
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=3, color_name = "Blue")

    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=3, color_name = "Green")

    base_C = sm.SE3(0.0, 0.0, 0.05)
    tower_C = Tower(base_pose=base_C, max_height=2, color_name = "Red")

    towers = [tower_A, tower_B, tower_C]

    return robots, bricks, towers

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
    
def plot_all_metrics_combined(panda_agent_1, panda_agent_2, global_limits_3d=None, start_time=None, end_time=None, figsize=(15,10), show=True, f=8):
    """
    Create an unique figure with all metrics and trajectories
    """
    fig = plt.figure(figsize=figsize) 
    
    # Define the grid 
    gs = GridSpec(7, 6, figure=fig, hspace=0.8, wspace=0.6)

    # Metrics (qd and cond) 
    # for robot 1
    ax_qd_1   = fig.add_subplot(gs[0, 0:3])  # Robot 1: q_dot (su 2 colonne)
    ax_cond_1 = fig.add_subplot(gs[1, 0:3], sharex=ax_qd_1) # Robot 1: kappa (sotto qd_1)
    # for robot 2
    ax_qd_2   = fig.add_subplot(gs[0, 3:6], sharey=ax_qd_1)  # Robot 2: q_dot (sulla stessa scala Y di qd_1)
    ax_cond_2 = fig.add_subplot(gs[1, 3:6], sharex=ax_qd_2) # Robot 2: kappa 
    
    # Height across time
    ax_height = fig.add_subplot(gs[2, 0:6]) # Altezza vs Tempo (su 3 colonne)
    # Inter-robot distance
    ax_distance = fig.add_subplot(gs[3, 0:6], sharex=ax_height)

    # 3d trajectory (Views XY, XZ, YZ) 
    ax_xy_traj = fig.add_subplot(gs[4:7, 0:2])  # Righe 4-6
    ax_xz_traj = fig.add_subplot(gs[4:7, 2:4])  # Righe 4-6
    ax_yz_traj = fig.add_subplot(gs[4:7, 4:6])  # Righe 4-6
    
    # Robot 1: Plot metrics on axes [ax_qd_1, ax_cond_1]
    panda_agent_1.plot_performance_metrics(axes=[ax_qd_1, ax_cond_1], qd_color='blue', cond_color='darkblue')
    # Robot 2: Plot metrics on axes [ax_qd_2, ax_cond_2]
    panda_agent_2.plot_performance_metrics(axes=[ax_qd_2, ax_cond_2], qd_color='orange', cond_color='darkorange')

    # 3D Trajectory
    axes_traj_3d = [ax_xy_traj, ax_xz_traj, ax_yz_traj]
    panda_agent_1.plot_3d_trajectory_views(axes=axes_traj_3d, line_color='blue', global_limits=global_limits_3d, f=f)
    panda_agent_2.plot_3d_trajectory_views(axes=axes_traj_3d, line_color='orange', global_limits=global_limits_3d, f=f)

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
                                                  end_time=end_time, marker_color='darkred')

    # Plot inter-robot distance over time
    plot_inter_robot_distance(panda_agent_1, panda_agent_2, ax_distance, 
                              start_time=start_time, end_time=end_time)

    ax_qd_1.set_xlabel('') 
    ax_qd_2.set_xlabel('') 
    ax_height.set_xlabel('')
    ax_distance.set_xlabel('Time [s]')
    
    for ax in axes_traj_3d:
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='lower right', fontsize=8)
    
    axes = [ax_cond_1, ax_cond_2, ax_height, ax_distance]
    for ax in axes:
        ax.legend(loc='lower right', fontsize=8)

    ax_cond_1.legend([])
    ax_cond_2.legend([])
    
    fig.suptitle('Collaborative Robotics: Performance and Trajectory Analysis', fontsize=18)
    
    if show:
        plt.show()

    return fig

def plot_inter_robot_distance(robot1, robot2, ax, start_time=None, end_time=None):
    """
    Plot the distance between two robots' end-effectors over time.
    """
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
    ax.plot(t_common, distances, linewidth=2, color='purple', label='Inter-Robot Distance')
    ax.axhline(0.2, linestyle='--', color='red', alpha=0.7, linewidth=2, 
               label='Safety Threshold (0.2m)')
    
    # Highlight regions below threshold
    below_threshold = distances < 0.2
    if np.any(below_threshold):
        ax.fill_between(t_common, 0, distances, where=below_threshold, 
                        alpha=0.3, color='red', label='Collision Risk Zone')
    
    ax.set_ylabel('Distance [m]')
    ax.set_title('Inter-Robot Distance Over Time', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.set_xlim(0, t_common[-1])

if __name__ == "__main__":
    # create the swift enviroment
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    
    # initialize the task
    robots, bricks, towers = initialize_task_4()

    # add elements to the environment
    for r in robots:
        r.register(env)

    for b in bricks:
        env.add(b.obj)

    # initialize the sensor
    sensor = Sensor(env, bricks=bricks, towers=towers, robots=robots)
    # initialize the controller of the robot  
    controller = Controller(env) 
    # initialize the task manager
    task_manager = TaskManager(env, sensor, robots=robots, controller=controller)

    start_time = time.time()
    task_manager.start()
    end_time=time.time()
    print(f'global time: {end_time-start_time}')

    global_range = global_limits(robots) 
    # fig = plot_all_metrics_combined(robots[0], robots[1], global_range, start_time, end_time, figsize=(17,20))
    # save_image(fig, 'plot_metrics_and_trajectory.png')
    
    fig = plot_all_metrics_combined(robots[0], robots[1], global_range, start_time, end_time, figsize=(18,14))
    #save_image(fig, 'plot_metrics_and_trajectory.png')

