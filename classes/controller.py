import swift
import spatialmath as sm
import spatialgeometry as sg
from classes.sensor import Sensor
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
    
def compute_cond_number(J_inv):
    """
    compute the condition number of the inverse jacobian matrix,
    it indicate how close a matrix to become non-invertible
    - low condition number -> robot moves normally
    - high condition number -> direction motion hard/impossible
    """
    return np.linalg.cond(J_inv)


MAX_SAFE_LIFT_HEIGHT = 0.6

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

    
    # general Sensor's requests

    def ask_free_bricks(self, sensor):# -> Brick:
        """
        ask to the Sensor class a Brick free to pick
        """
        return sensor.get_free_bricks()
    
    def ask_towers(self, sensor):
        """
        ask to the Sensor class the towers
        """
        return sensor.get_towers()

    def search_uncomplete_tower(self, sensor):
        """
        search between the towers in the environment one that is incomplete
        """
        #for t in self.ask_towers(sensor):
        #    if not t.is_complete():
        #        return t
        #return None
        towers = sensor.get_incomplete_towers()
        return towers[0] if towers else None

    def free_brick(self, sensor):
        """
        return a brick free
        """
        # free_bricks = self.ask_free_bricks(sensor)
        # if len(free_bricks) > 0:
        #   return free_bricks.pop(0)
        # return None
        free_bricks = sensor.get_free_bricks()
        return free_bricks[0] if free_bricks else None
    


    # internal methods

    def select_brick_and_tower(self, sensor: 'Sensor', robot_name):
        """
        select a free brick and a tower to complete from the sensor environent
        """
        #brick_to_pick: Brick | None = self.free_brick(sensor)
        #target_tower: Tower | None = self.search_uncomplete_tower(sensor)
        free_bricks = sensor.get_free_bricks()
        incomplete_towers = sensor.get_incomplete_towers()

        if robot_name == "panda":
            free_bricks = [b for b in free_bricks if b.pose()[0, 3] <= 0.05]
        elif robot_name == "panda_2":
            free_bricks = [b for b in free_bricks if b.pose()[0, 3] > 0.05]
    
    
        #try to lock both resources
        for brick in free_bricks:
            if brick.try_lock(robot_name):
                for tower in incomplete_towers:
                    if tower.try_lock(robot_name):
                        return brick, tower
                # Se non trovo tower, rilascio il brick
                brick.unlock(robot_name)
    
        return None, None
        #return brick_to_pick, target_tower
    
    def generate_path_points(self, brick_to_pick: Brick | None, target_tower: Tower | None):
        """
        returns a list of points that define the safe path to do
        """
        def set_global_height(T: sm.SE3, height: float) -> sm.SE3:
            # since .copy() doesn't work
            new_T = sm.SE3(T.A.copy())     # COPIA VERA
            new_T.t[2] = height
            return new_T

        if not brick_to_pick or not target_tower:
            print("Controller: resources or target are missing")
            return None

        T_release_target = target_tower.get_next_pose()

        # 1. pick the brick
        T_pick        = sm.SE3(brick_to_pick.pose())
        # 0. safe pick pose
        T_pick_safe   = set_global_height(T_pick, self.max_safe_height)
        # 2. go up until safe height
        T_lift        = set_global_height(T_pick, self.max_safe_height)

        # 3. shift on the tower
        T_transfer    = set_global_height(T_release_target, self.max_safe_height)
        # 4. correct height where release the brick
        T_release     = sm.SE3(T_release_target.A.copy())
        
        T_end         = set_global_height(T_release_target, self.max_safe_height)

        return [T_pick_safe, T_pick, T_lift, T_transfer, T_release, T_end]


    # Robot's movements
    def compute_qdot(self, robot: Robot_arm, target: sm.SE3):
        """
        compute the position of the joints
        """
        r_name = robot.name 
        robot = robot._robot
        T = robot.fkine(robot.q)
        error = target.t - T.t
        if r_name == "panda_2":
            error_bot = error.copy()
            error_bot[0] = -error[0]
            error_bot[1] = -error[1]
        J = robot.jacob0(robot.q)[0:3,:]
        J_inv = damped_pseudoinverse(J)
        cond_number = compute_cond_number(J_inv)
        if r_name == "panda_2":
            qdot = self.gain*J_inv@error_bot
            return qdot, error_bot, cond_number
        
        qdot = self.gain*J_inv@error
        return qdot, error, cond_number
        
    def drag_brick(self, brick, robot:rtb.ERobot):
        """
        the robot drag the brick as the arm move
        """
        pos = robot.fkine(robot.q)
        brick.update_position(pos)
    
    def move_to_pose(self, agent: Robot_arm, target_pose: sm.SE3, brick: Brick = None, dt: float = 0.01, tol: float = 0.001):
        """
        moves the robot end-effector to a target pose,
        if brick is specified, the robot drags it
        """
        error = np.inf 
        while np.linalg.norm(error) >= tol:
            qdot, error, cond_number = self.compute_qdot(agent, target_pose)
            agent.apply_velocity_cmd(qdot, cond_number, error)
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
        brick_to_pick, target_tower = self.select_brick_and_tower(sensor, agent.name)

        # calculate the path points
        way_points = self.generate_path_points(brick_to_pick, target_tower)

        if way_points is None:
            return None
        
        # go to pick the brick
        self.move_to_pose(agent, way_points[0], dt = dt)
        self.move_to_pose(agent, way_points[1], dt = dt)
        
        # move and place the brick
        for target_point in way_points[2:-1]:
             self.move_to_pose(agent, target_point, brick_to_pick, dt = dt)

        # fake parallel placing of the brick
        brick_to_pick.placing_orientation()

        # move to safe height
        self.move_to_pose(agent, way_points[-1], dt = dt)
        
        self.__env.step(dt)

        # increasing the counter of the bricks on the tower
        target_tower.add(brick_placed=brick_to_pick)
        # flag the brick as placed
        brick_to_pick.placed = True

        #realase the locsk 
        brick_to_pick.unlock(agent.name)
        target_tower.unlock(agent.name)





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