import swift
import spatialmath as sm
import spatialgeometry as sg
#from classes.sensor import Sensor
#from classes.objects import Tower, FictionalBrick
import numpy as np
import roboticstoolbox as rtb
# from classes import *
from classes.robot import Robot_arm
from classes.objects import Tower, Brick


def damped_pseudoinverse(J: np.ndarray):
    """
    calculate the damped pseudoinverse of the jacobian matrix
    """
    m, _ = J.shape
    return J.T @ np.linalg.inv(J@J.T + 0.01 * np.eye(m))
    

# fittizi
MAX_SAFE_LIFT_HEIGHT = 0.6

TOWER_BASE_A = sm.SE3(0.0, 0.0, 0.05) 
TOWER_BASE_B = sm.SE3(0.0, -0.5, 0.0)
tower_A = Tower(TOWER_BASE_A, max_height=2, name="Tower_1")
tower_B = Tower(TOWER_BASE_B, max_height=0, name="Tower_2")
TOWERS = [tower_A, tower_B] 
brick_A = Brick(sm.SE3(0.0, 0.5, 0.05))
brick_B = Brick(start_pose=sm.SE3(0.1, 0.5, 0.05))
FREE_BRICKS = [brick_B, brick_A]  

class Controller:
    """
    class that include the logic of the Robot's movement
    Collaborative Task Manager
    """
    def __init__(self, env: swift.Swift): #,sensor: Sensor =None):
        self.__env = env
        #self.__sensor = sensor
        self.max_bricks_in_towel = 10
        self.max_safe_height = MAX_SAFE_LIFT_HEIGHT
        self.gain = 1.5

        for BRICK in FREE_BRICKS:
            #brick = Brick(sm.SE3(0.0, 0.5, 0.05))
            self.__env.add(BRICK.obj)
    

    def compute_qdot(self, robot: rtb.ERobot, target: sm.SE3):
        """
        compute the position of the joints
        """
        T = robot.fkine(robot.q)
        error = target.t - T.t
        J = robot.jacob0(robot.q)[0:3,:]
        J_inv = damped_pseudoinverse(J)
        qdot = self.gain*J_inv@error
        
        return qdot, error
        
    def drag_brick(self, brick, robot:rtb.ERobot):
        """
        the robot drag the brick as the arm move
        """
        pos = robot.fkine(robot.q)
        brick.update_position(pos)

    def ask_free_bricks(self, sensor):# -> Brick:
        """
        ask to the Sensor class a Brick free to pick
        """
        # TODO by Sensor developer
        return FREE_BRICKS
    
    def ask_towers(self, sensor):
        """
        ask to the Sensor class the towers
        """
        # TODO by Sensor developer
        return TOWERS

    def search_uncomplete_tower(self, sensor):
        """
        search between the towers in the environment one that is incomplete
        """
        for t in self.ask_towers(sensor):
            if not t.is_complete():
                return t
        return None
    
    def free_brick(self, sensor):
        """
        return a brick free
        """
        free_bricks = self.ask_free_bricks(sensor)
        if len(free_bricks) > 0:
            return free_bricks.pop(0)
        return None
    
    def select_brick_and_tower(self, sensor: 'Sensor'):
        """
        select a free brick and a tower to complete from the sensor environent
        """
        brick_to_pick: Brick | None = self.free_brick(sensor)
        target_tower: Tower | None = self.search_uncomplete_tower(sensor)
        
        return brick_to_pick, target_tower
    
    def generate_path_points(self, brick_to_pick: Brick | None, target_tower: Tower | None):
        """
        returns a list of points that define the safe path to do
        """
        if not brick_to_pick or not target_tower:
            print("Controller: resources or target are missing")
            return None
        
        T_release_target = target_tower.get_next_pose()

        # 1. pick the brick
        T_pick = brick_to_pick.start_pose
        # 2. go up until safe height
        T_lift = T_pick @ sm.SE3([0, 0, self.max_safe_height])
        # 3. shift on the tower
        T_transfer = T_release_target @ sm.SE3([0, 0, self.max_safe_height])
        # 4. correct height where release the brick
        T_release = T_release_target

        # return poses
        return [T_pick, T_lift, T_transfer, T_release]
    
    def move_to_pose(self, agent: Robot_arm, target_pose: sm.SE3, brick: Brick = None, dt: float = 0.01, tol: float = 0.001):
        """
        moves the robot end-effector to a target pose,
        if brick is specified, the robot drags it
        """
        error = np.inf 
        while np.linalg.norm(error) >= tol:
            qdot, error = self.compute_qdot(agent._robot, target_pose)
            agent.apply_velocity_cmd(qdot)
            if brick is not None:
                self.drag_brick(brick, agent._robot)
            self.__env.step(dt)

    def pick_and_place(self, agent: Robot_arm, sensor: 'Sensor', dt: float = 0.01):
        """
        given the agent and the sensor:
        1. selects a free brick and a tower to complete
        2. calculates the points to follow to complete the task
        3. moves the arm to make the task
        """
        # select a brick to pick and a tower to complete
        brick_to_pick, target_tower = self.select_brick_and_tower(sensor)

        # calcuate the path points
        way_points = self.generate_path_points(brick_to_pick, target_tower)

        if way_points is None:
            return None
        
        # go to pick the brick
        self.move_to_pose(agent, way_points[0], dt = dt)

        # move and place the brick
        for target_point in way_points[1:]:
             self.move_to_pose(agent, target_point, brick_to_pick, dt = dt)

        # fake parallel placing of the brick
        brick_to_pick.placing_orientation()
        self.__env.step(dt)

        # increasing the counter of the bricks on the tower
        target_tower.add(brick_placed=brick_to_pick)







# # TODO: in future for task manager
# def execute_stacking_sequence(self, robot, brick, target_pose):
#     # calculate complete path
#     sequence_of_poses = self._calculate_full_path(brick.start_pose, target_pose)
#     # avoid collisions
#     for pose in sequence_of_poses:
#         if self.is_path_safe(robot, pose):
#             robot.move_to_pose(pose)
#         else:
#             print(f"{robot.name}: STOPPED MOVEMENT TO AVOID COLLISION")
#             return
    
#     robot.is_busy = False

# def find_available_robot(self) -> Robot_arm | None:
#     """
#     Find first robot not busy to complete the task
#     """
#     for robot in self.__robots:
#         if not robot.is_busy:
#             robot.is_busy = True
#             return robot
#     return None