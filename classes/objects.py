import spatialmath as sm
import spatialgeometry as sg

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
    

class Brick:

    def __init__(self, start_pose, scale=[0.1, 0.2, 0.1], color=[0.0, 0.1, 0.9, 0.1]):
        self.start_pose = start_pose
        self.scale = scale
        self.color = color
        self.obj = sg.Cuboid(scale=self.scale, pose=self.start_pose, color=self.color)

        self.height = self.scale[2]
