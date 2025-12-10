import spatialmath as sm
import spatialgeometry as sg
import numpy as np

class Tower:
    def __init__(self, base_pose: sm.SE3, max_height: int, color_name: str, name: str = 'Tower'):
        self.name = name
        self.base_pose = base_pose
        self.color_name = color_name.lower()
        self._current_logical_height = 0.0
        self.max_height = max_height
        self.bricks = []
        self.lock = False
        self.locked_by = None

    def try_lock(self, robot_name: str) -> bool:
        """ 
        Try to lock the tower for a specific robot 
        """
        if not self.lock:
            self.lock = True
            self.locked_by = robot_name
            return True
        return False
    
    def unlock(self, robot_name: str) -> bool:
        """ 
        Unlock the tower if it is locked by the specific robot 
        """
        if self.lock and self.locked_by == robot_name:
            self.lock = False
            self.locked_by = None
    
    def add(self, brick_placed: 'Brick'):
        """
        Increment height 
        """
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
        """
        Return whether the tower is complete.
        """
        return len(self.bricks) >= self.max_height
    

class Brick:

    def __init__(self, start_pose: sm.SE3, scale=[0.1, 0.2, 0.1], color_name="blue", name: str = 'Brick'):
        self.name = name
        self.start_pose = start_pose 
        self.scale = scale
        self.color_name = color_name.lower()
        colors_dict = {"red":[1.0, 0.0, 0.0, 0.1], "green":[0.0, 1.0, 0.0, 0.1], "blue":[0.0, 0.0, 1.0, 0.1], "yellow":[1.0, 1.0, 0.0, 0.1]}
        self.color = colors_dict[self.color_name.lower()]
        self.obj = sg.Cuboid(scale=self.scale, pose=self.start_pose, color=self.color)

        self.height = self.scale[2]
        self.placed = False

        self.lock = False
        self.locked_by = None
    
    def pose(self):
        """
        the current pose of the Cuboid
        """
        return self.obj.T
    
    def update_position(self, pos):
        """
        match the position of the brick to the one of the end-factor
        """
        self.obj.T = pos
        self.start_pose = pos

    def placing_orientation(self):
        """
        modify the orientation of the brick to make it parallel to the plane
        """
        I_R = np.eye(3)
        new_T = self.obj.T.copy()
        new_T[:3, :3] = I_R
        self.update_position(new_T)

    def try_lock(self, robot_name: str) -> bool:
        """ 
        Try to lock the brick for a specific robot 
        """
        if not self.lock:
            self.lock = True
            self.locked_by = robot_name
            return True
        return False
    
    def unlock(self, robot_name: str) -> bool:
        """ 
        Unlock the brick if it is locked by the specific robot 
        """
        if self.lock and self.locked_by == robot_name:
            self.lock = False
            self.locked_by = None