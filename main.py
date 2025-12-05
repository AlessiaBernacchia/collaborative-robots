from classes.robot import *
from classes.controller import *
from classes.objects import *
from time import time
# from classes.sensor import *
# from classes import *

if __name__ == "__main__":
    #create the swift enviroment
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    
    # initialize the robot arm
    panda_agent = Robot_arm("panda")
    panda_agent_2 = Robot_arm("panda_2", sm.SE3.Rz(np.pi))
    panda_agent.set_position(-0.70, 0.0, 0.0)
    panda_agent_2.set_position(0.70, 0.0, 0.0)
    panda_agent_2.modify_orientation_base()
    panda_agent.register(env)
    panda_agent_2.register(env)

    # initialize bricks
    brick_A = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_B = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_C = Brick(sm.SE3(0.11, 0.5, 0.15))
    brick_D = Brick(sm.SE3(0.11, 0.5, 0.05))

    brick_E = Brick(sm.SE3(0.22, 0.5, 0.15))
    brick_F = Brick(sm.SE3(0.22, 0.5, 0.05))
    brick_G = Brick(sm.SE3(-0.11, 0.5, 0.15))
    brick_H = Brick(sm.SE3(-0.11, 0.5, 0.05))

    bricks = [brick_A, brick_B, brick_C, brick_D, brick_E, brick_F, brick_G, brick_H]

    # add bricks to the environment
    for b in bricks:
        env.add(b.obj)

    # initialize the tower
    base_A = sm.SE3(0.0, 0.2, 0.05)
    tower_A = Tower(base_pose=base_A, max_height=1)
    
    base_B = sm.SE3(0.0, -0.2, 0.05)
    tower_B = Tower(base_pose=base_B, max_height=1)
    
    towers = [tower_A, tower_B]

    # initialize the controller of the robot
    controller = Controller(env)       

    # initialize the sensor
    sensor = Sensor(env, bricks=bricks, towers=towers, robots=[panda_agent])
    
    # start simulation by iterating each brick in the list
    for _ in range(len(bricks)):
        #panda_agent.start_task()
        panda_agent_2.start_task()
        #controller.pick_and_place(panda_agent, sensor)
        controller.pick_and_place(panda_agent_2, sensor)
        #panda_agent.task_completed()
        panda_agent_2.task_completed()        

    panda_agent.plot_performance_metrics()

    panda_agent.plot_3d_trajectory_views()
    panda_agent.plot_height_over_time()

        
