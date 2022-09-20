from motion_planning_env.free_collision_obstacle import FreeCollisionObstacleConfig, FreeCollisionObstacle

from motion_planning_scene_helpers.motion_planning_component import (
    DimensionNotSuitableForEnv
)

from dataclasses import dataclass, KW_ONLY
from omegaconf import OmegaConf

@dataclass
class GeometryConfig:
    """Configuration dataclass for geometry for urdf obstacles.

    This configuration class holds information about the urdf file.

    Parameters:
    ------------

    urdf : str : Filename of the urdf
    """
    urdf: str

@dataclass
class UrdfObstacleConfig(FreeCollisionObstacleConfig):
    """
    Configuration dataclass for urdf obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    scaling: float : Scaling factor of obstacle
    """
    geometry: GeometryConfig
    _: KW_ONLY
    scaling: float

class UrdfObstacle(FreeCollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(UrdfObstacleConfig)
        super().__init__(schema, **kwargs)

    def urdf(self):
        return self._config.geometry.urdf

    def scaling(self):
        return self._config.scaling

    def add_to_bullet(self, pybullet):
        if self.dimension() != 3:
            raise DimensionNotSuitableForEnv(
                "Pybullet only supports two dimensional obstacles"
            )
        pybullet.loadURDF(
                fileName=self.urdf(),
                basePosition=self.position(),
                baseOrientation=self.orientation(),
                globalScaling=self.scaling()
                )
