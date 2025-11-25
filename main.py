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
    
    #initialize the controller of the robot
    controller = Controller(env)
        
    #initialize the tower
    base = sm.SE3(0.0, 0.0, 0.05)
    tower = Tower(base_pose=base, max_height=2)
    
    #start simulation by iterating each brick in the list
    for _ in range(2):
        controller.pick_and_place(panda_agent, 'sensor')

        
        
        
