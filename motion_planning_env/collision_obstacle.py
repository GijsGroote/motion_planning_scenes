from abc import abstractmethod
from motion_planning_scene_helpers.motion_planning_component import MotionPlanningComponent

from dataclasses import dataclass

@dataclass
class CollisionObstacleConfig:
    """Configuration dataclass for sphere obstacle.

    This configuration class holds information about type
    of collision obstacle.

    Parameters:
    ------------

    type : str : Type of the obstacle
    """
    type: str


class CollisionObstacle(MotionPlanningComponent):

    def type(self):
        return self._config.type

    @abstractmethod
    def position(self, **kwargs):
        pass

    def update_bullet_position(self, pybullet, **kwargs):
        pass
