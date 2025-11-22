import swift
import spatialmath as sm
import spatialgeometry as sg
#import Sensor from sensor

# fittizi
MAX_SAFE_LIFT_HEIGHT = 0.6

class FictionalBrick:
    # 
    def __init__(self, start_pose, scale=[0.1, 0.2, 0.1], color=[0.0, 0.1, 0.9, 0.1]):
        self.start_pose = start_pose
        self.scale = scale
        self.color = color
        self.obj = sg.Cuboid(scale=self.scale, pose=self.start_pose, color=self.color)

        self.height = self.scale[2]

brick_A = FictionalBrick(start_pose=sm.SE3(0.5, 0.5, 0.05))
brick_B = FictionalBrick(start_pose=sm.SE3(0.5, 0.25, 0.05))
FREE_BRICKS = [brick_A, brick_B]  

class Tower:
    def __init__(self, base_pose: sm.SE3, max_height: int):
        self.base_pose = base_pose
        self._current_logical_height = 0.0
        self.max_height = max_height
        self.bricks = []
    
    def add(self, brick_placed: 'Brick'):
        """Increment height """
        self.bricks.append(brick_placed)
        self._current_logical_height += brick_placed.height

    def get_next_pose(self) -> sm.SE3: #, brick_to_add: 'Brick') -> sm.SE3:
        """
        Next pose (end-effector) for next brick 
        actual_pose + next_brick_neight
        """
        #height = self._current_logical_height + brick_to_add.height
        #return self.base_pose @ sm.SE3([0, 0, height])
        return self.base_pose @ sm.SE3([0, 0, self._current_logical_height])

        
    def is_complete(self) -> bool:
        return len(self.bricks) >= self.max_height

TOWER_BASE_A = sm.SE3(0.0, 0.0, 0.0) 
TOWER_BASE_B = sm.SE3(0.0, -0.5, 0.0)
tower_A = Tower("Tower_1", TOWER_BASE_A, max_height=3)
tower_B = Tower("Tower_2", TOWER_BASE_B, max_height=0)
TOWERS = [tower_A, tower_B] 

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
    
    def generate_path_points(self, sensor):
        """
        returns a list of points that define the safe path to do
        """
        brick_to_pick = self.ask_free_brick()
        target_tower = self.search_uncomplete_tower()

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

        target_tower.add(brick_to_pick)

        # return poses
        return [T_pick, T_lift, T_transfer, T_release]
    
        
    
# TODO: by Robot developer
# obtain path_points = controller.generate_path_points()
# execute it robot.execute_path(path_points)







    # TODO: in future for task manager
    def execute_stacking_sequence(self, robot, brick, target_pose):
        # calculate complete path
        sequence_of_poses = self._calculate_full_path(brick.start_pose, target_pose)
        # avoid collisions
        for pose in sequence_of_poses:
            if self.is_path_safe(robot, pose):
                robot.move_to_pose(pose)
            else:
                print(f"{robot.name}: STOPPED MOVEMENT TO AVOID COLLISION")
                return
        
        robot.is_busy = False

    def find_available_robot(self) -> FictionalRobot | None:
        """
        Find first robot not busy to complete the task
        """
        for robot in self.__robots:
            if not robot.is_busy:
                robot.is_busy = True
                return robot
        return None