from classes.robot import *
from classes.controller import *
from classes.objects import *
#from classes.sensor import *


if __name__ == "__main__":
    #create the swift enviroment
    env = swift.Swift()
    env.launch(realtime=True, comms="rtc", browser="browser")
    
    # initialize the robot arm
    panda_agent = Robot_arm("panda")
    panda_agent.set_position(-0.5, 0.0, 0.0)
    panda_agent.register(env)
    
    #initialize the controller of the robot
    robot_control = Controller(env)
    dt = 0.01
    
    #create a list of bricks
    bricks = []
    for i in range(2):
        brick = Brick(sm.SE3(0.0, 0.5, 0.05))
        env.add(brick.obj)
        bricks.append(brick)
        
    #initialize the tower
    base = sm.SE3(0.0, 0.0, 0.05)
    tower = Tower(base_pose=base, max_height=2)


    #start simulation by iterating each brick in the list
    for brick in bricks:
        target_brick = brick.start_pose
        target_tower = tower.get_next_pose() #retrieve the right high where the next brick will be placed
        
        #retrieving the brick
        error_brick = np.inf 
        while np.linalg.norm(error_brick) >= 0.001:
            qdot, error_brick = robot_control.compute_qdot(panda_agent._robot, target_brick)
            panda_agent.apply_velocity_cmd(qdot)
            env.step(dt)
        
        #placing the brick on the tower
        error_tower = np.inf
        while np.linalg.norm(error_tower) >= 0.001:
            qdot, error_tower = robot_control.compute_qdot(panda_agent._robot, target_tower)
            panda_agent.apply_velocity_cmd(qdot)
            robot_control.drag_brick(brick, panda_agent._robot)
            env.step(dt)
            
        #fake parallel placing of the brick
        brick.placing_orientation()
        env.step(dt)

        #increasing the counter of the bricks on the tower
        tower.add(brick)
        
        #checking if the tower is finish
        if tower.is_complete():
            print("Tower finished")

        
        
        
