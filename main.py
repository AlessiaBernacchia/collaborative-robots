from classes.robot import *
from classes.controller import *
from classes.objects import *
# from classes.sensor import *
# from classes import *

if __name__ == "__main__":
    #create the swift enviroment
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    
    # initialize the robot arm
    panda_agent = Robot_arm("panda")
    panda_agent.set_position(-0.5, 0.0, 0.0)
    panda_agent.register(env)

    # initialize bricks
    brick_A = Brick(sm.SE3(0.0, 0.5, 0.15))
    brick_B = Brick(sm.SE3(0.0, 0.5, 0.05))
    brick_C = Brick(sm.SE3(0.1, 0.5, 0.15))
    brick_D = Brick(sm.SE3(0.1, 0.5, 0.05))

    bricks = [brick_A, brick_B, brick_C, brick_D]

    # add bricks to the environment
    for b in bricks:
        env.add(b.obj)

    # initialize the tower
    base = sm.SE3(0.0, 0.0, 0.05)
    tower = Tower(base_pose=base, max_height=4)
    
    # initialize the controller of the robot
    controller = Controller(env)       

    # initialize the sensor
    sensor = Sensor(env, bricks=bricks, towers=[tower], robots=[panda_agent])
    
    # start simulation by iterating each brick in the list
    for _ in range(len(bricks)):
        controller.pick_and_place(panda_agent, sensor)

        
        
        
