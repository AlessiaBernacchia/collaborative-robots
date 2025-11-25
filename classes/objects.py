import spatialmath as sm
import spatialgeometry as sg
import numpy as np

class Tower:
    def __init__(self, base_pose: sm.SE3, max_height: int, name: str = 'Tower'):
        self.name = name
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
    

class Brick:

    def __init__(self, start_pose: sm.SE3, scale=[0.1, 0.2, 0.1], color=[0.0, 0.1, 0.9, 0.1], name: str = 'Brick'):
        self.name = name
        self.start_pose = start_pose
        self.scale = scale
        self.color = color
        self.obj = sg.Cuboid(scale=self.scale, pose=self.start_pose, color=self.color)

        self.height = self.scale[2]
    
    def update_position(self, pos):
        """
        match the position of the brick to the one of the end-factor
        """
        self.obj.T = pos

    def placing_orientation(self):
        """
        modify the orientation of the brick to make it parallel to the plane
        """
        I_R = np.eye(3)
        new_T = self.obj.T.copy()
        new_T[:3, :3] = I_R
        self.obj.T = new_T