import swift
import spatialmath as sm
import spatialgeometry as sg

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
    Translates high-level tasks into low-level joint velocity commands 
    and executes movement, checking precedence with the Task Manager during conflicts.
    """
    def __init__(self, env: swift.Swift, gain: float = 4):
        self.__env = env
        self.max_safe_height = MAX_SAFE_LIFT_HEIGHT
        self.gain = gain    
    
    def generate_path_points(self, brick_to_pick: Brick | None, target_tower: Tower | None = None):
        """
        returns a list of points that define the safe path to do
        """
        def set_global_height(T: sm.SE3, height: float) -> sm.SE3:
            # since .copy() doesn't work
            new_T = sm.SE3(T.A.copy())     # COPIA VERA
            new_T.t[2] = height
            return new_T

        if not brick_to_pick:
            print(f"Controller: resources are missing")
            return None

        if not target_tower:
            # 1. pick the brick
            T_pick        = sm.SE3(brick_to_pick.pose())
            # 0. safe pick pose
            T_pick_safe   = set_global_height(T_pick, self.max_safe_height)
            # 2. go up until safe height
            T_lift        = set_global_height(T_pick, self.max_safe_height)
            return [T_pick_safe, T_pick, T_lift]

        T_release_target = target_tower.get_next_pose()
        # 3. shift on the tower
        T_transfer    = set_global_height(T_release_target, self.max_safe_height)
        # 4. correct height where release the brick
        T_release     = sm.SE3(T_release_target.A.copy())
        
        T_end         = set_global_height(T_release_target, self.max_safe_height)

        return [T_transfer, T_release, T_end]

    
    # Robot's movements
    def rest(self, robot:Robot_arm, task_manager, dt=0.01):
        """
        move the robot to the rest position
        """
        rest_q = robot.rest_qr()
        rest_pose = robot._robot.fkine(rest_q)
        self.move_to_pose(robot, rest_pose,task_manager, dt=dt)
        
    def compute_qdot(self, robot: Robot_arm, target: sm.SE3):
        """
        compute the position of the joints
        """
        r_name = robot.name 
        robot = robot._robot
        T = robot.fkine(robot.q)
        error = target.t - T.t
        J = robot.jacob0(robot.q)[0:3,:]
        J_inv = damped_pseudoinverse(J)
        cond_number = compute_cond_number(J_inv)
        
        if r_name == "panda_2":
            error_bot = error.copy()
            error_bot[0] = -error[0]
            error_bot[1] = -error[1]
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
    
    def move_to_pose(self, agent: Robot_arm, target_pose: sm.SE3, task_manager, brick: Brick = None, dt: float = 0.01, tol: float = 0.001):
        """
        moves the robot end-effector to a target pose,
        if brick is specified, the robot drags it
        Moves the robot end-effector to a target pose, managing anti-collision 
        precedence at every step.
        """
        error = np.inf 
        # move until is close to the target
        while agent.distance >= tol:
            # simulate next movement to reach the target point
            qdot_initial, error, cond_number = self.compute_qdot(agent, target_pose)
            error_norm = np.linalg.norm(error)
            # ask to task maanger if he can move safely
            can_move = task_manager.resolve_collision_precedence(agent, error_norm)
            # select the q for the movement
            qdot_final = qdot_initial

            # if not can_move:
            #     qdot_avoidance, error, _ = self.compute_qdot(agent, target_pose@agent._safe_transition)
            #     qdot_final = qdot_avoidance
            # # move
            
            # if it cannot move, the robot will stay away from the danger area.
            waiting_counter = 0
            waiting_t = 100*dt
            while not can_move and waiting_counter <= waiting_t:
                qdot_avoidance, error_avoidance, cond_number_avoidance = self.compute_qdot(agent, target_pose@agent._safe_transition)
                agent.apply_velocity_cmd(qdot_avoidance, cond_number_avoidance, error_avoidance)
                
                if brick is not None:
                    self.drag_brick(brick, agent._robot)
                    
                self.__env.step(dt)
                waiting_counter += dt
            
            # reset the distance and start next iteration if it entered 
            # in the avoidance procedure
            if waiting_counter >= waiting_t:
                agent.reset_distance()
                continue

            agent.apply_velocity_cmd(qdot_final, cond_number, error)
            
            if brick is not None:
                self.drag_brick(brick, agent._robot)

            self.__env.step(dt)
            
        agent.reset_distance()