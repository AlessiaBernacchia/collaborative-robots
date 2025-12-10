from classes.robot import *
from classes.controller import *
from classes.objects import *
from classes.task_manager import *

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
# from classes.sensor import *
# from classes import *
import threading
import time
import os

PLOTS_DIR = './plots'

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
    gs = GridSpec(5, 6, figure=fig, hspace=1, wspace=0.6)

    # Metrics (qd and cond) 
    # for robot 1
    ax_qd_1   = fig.add_subplot(gs[0, 0:3])  # Robot 1: q_dot (su 2 colonne)
    ax_cond_1 = fig.add_subplot(gs[1, 0:3], sharex=ax_qd_1) # Robot 1: kappa (sotto qd_1)
    # for robot 2
    ax_qd_2   = fig.add_subplot(gs[0, 3:6], sharey=ax_qd_1)  # Robot 2: q_dot (sulla stessa scala Y di qd_1)
    ax_cond_2 = fig.add_subplot(gs[1, 3:6], sharex=ax_qd_2) # Robot 2: kappa 
    
    # Height across time
    ax_height = fig.add_subplot(gs[2, 0:6]) # Altezza vs Tempo (su 3 colonne)

    # 3d trajectory (Views XY, XZ, YZ)
    ax_xy_traj = fig.add_subplot(gs[3:5, 0:2]) # Top View
    ax_xz_traj = fig.add_subplot(gs[3:5, 2:4], sharex=ax_xy_traj) # Side View
    ax_yz_traj = fig.add_subplot(gs[3:5, 4:6], sharex=ax_xy_traj) # Front View
    
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
                                        start_time=start_time, end_time=end_time, global_limits=global_limits_3d, 
                                        f=f)
    panda_agent_2.plot_height_over_time(ax=ax_height, line_color='orange', 
                                        start_time=start_time, end_time=end_time, global_limits=global_limits_3d, 
                                        f=f)

    ax_qd_1.set_xlabel('') 
    ax_qd_2.set_xlabel('') 
    ax_height.set_xlabel('Time [s]')
    
    for ax in axes_traj_3d:
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='lower right')
    
    axes = [ax_cond_1, ax_cond_2, ax_height]
    for ax in axes:
        ax.legend(loc='lower right')

    ax_cond_1.legend([])
    ax_cond_2.legend([])
    
    fig.suptitle('Collaborative Robotics: Performance and Trajectory Analysis', fontsize=18)
    
    if show:
        plt.show()

    return fig

if __name__ == "__main__":
    #create the swift enviroment
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    
    # initialize the robot arm
    panda_agent = Robot_arm("panda", sm.SE3.Rz(0), sm.SE3.Tx(-0.2))
    panda_agent_2 = Robot_arm("panda_2", sm.SE3.Rz(np.pi), sm.SE3.Tx(0.2))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent_2.set_position(0.70, 0.0, 0.0)
    panda_agent.modify_orientation_base()
    panda_agent_2.modify_orientation_base()
    panda_agent.register(env)
    panda_agent_2.register(env)

    robots = [panda_agent, panda_agent_2]

    # initialize bricks
    brick_A = Brick(sm.SE3(0.0, 0.5, 0.15), color_name="Green")
    brick_B = Brick(sm.SE3(0.0, 0.5, 0.05), color_name="Red")
    brick_C = Brick(sm.SE3(0.11, 0.5, 0.15), color_name="Blue")
    brick_D = Brick(sm.SE3(0.11, 0.5, 0.05), color_name="Blue")

    brick_E = Brick(sm.SE3(0.22, 0.5, 0.15), color_name="green")
    brick_F = Brick(sm.SE3(0.22, 0.5, 0.05), color_name="green")
    brick_G = Brick(sm.SE3(-0.11, 0.5, 0.15), color_name="Red")
    brick_H = Brick(sm.SE3(-0.11, 0.5, 0.05), color_name="Blue")
    

    bricks = [brick_A, brick_B, brick_C, brick_D, brick_E, brick_F, brick_G, brick_H]

    # add bricks to the environment
    for b in bricks:
        env.add(b.obj)

    # initialize the tower
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=3, color_name = "Blue")

    
    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=3, color_name = "Green")

    base_C = sm.SE3(0.0, 0.0, 0.05)
    tower_C = Tower(base_pose=base_C, max_height=2, color_name = "Red")

    towers = [tower_A, tower_B, tower_C]

    # initialize the sensor
    sensor = Sensor(env, bricks=bricks, towers=towers, robots=robots)
    # initialize the controller of the robot  
    controller = Controller(env) 
    # initialize the task manager
    task_manager = TaskManager(env, sensor, robots=robots, controller=controller)
    
    
    # start simulation by iterating each brick in the list
    # for _ in range(len(bricks)):
    #     panda_agent.start_task()
    #     panda_agent_2.start_task()
    #     controller.pick_and_place(panda_agent, sensor)
    #     controller.pick_and_place(panda_agent_2, sensor)
    #     panda_agent.task_completed()
    #     panda_agent_2.task_completed()        

    # panda_agent.plot_performance_metrics()

    # panda_agent.plot_3d_trajectory_views()
    # panda_agent.plot_height_over_time()

    def robot_worker(robot, controller, sensor, max_tasks):
        """
        Worker function that executes tasks for a single robot.
        Each robot will try to complete max_tasks bricks.
        """
        tasks_completed = 0
        
        while tasks_completed < max_tasks:
            robot.start_task()
            result = task_manager.place_one_brick(robot, dt=0.01)
            # result = controller.pick_and_place(robot, sensor)
            robot.task_completed()
            
            if result is None:  # Task completed successfully
                tasks_completed += 1
            else:
                time.sleep(0.5)
            if tasks_completed == max_tasks:
                controller.rest(robot, task_manager, dt=0.01)
    
    # Create two threads, one for each robot
    # Each robot will complete 4 bricks (8 total / 2 robots)
    start_time = time.time()
    thread1 = threading.Thread(
        target=robot_worker,
        args=(panda_agent, controller, sensor, 4)
    )
    thread2 = threading.Thread(
        target=robot_worker,
        args=(panda_agent_2, controller, sensor, 4)
    )
    
    # Start both robots simultaneously
    thread1.start()
    thread2.start()
    
    # Wait for both robots to complete their tasks
    thread1.join()
    thread2.join()
    
    end_time=time.time()
    

    global_range = global_limits(robots) 
    fig = plot_all_metrics_combined(panda_agent, panda_agent_2, global_range, start_time, end_time, figsize=(17,20))
    # save_image(fig, 'plot_metrics_and_trajectory.png')



            
