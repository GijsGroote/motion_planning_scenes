from dataclasses import dataclass
import os
from motion_planning_env.free_collision_obstacle import (
        FreeCollisionObstacleConfig,
        FreeCollisionObstacle
        )

from motion_planning_scene_helpers.motion_planning_component import (
    DimensionNotSuitableForEnv,
)

from omegaconf import OmegaConf

@dataclass
class GeometryConfig:
    """
    This configuration class holds information about
    the geometry of a box obstacle.
    Parameters:
    ------------
    length: float: Lenght of the obstacle
    width: float: Width of the obstacle
    height: float: Height of the obstacle
    """
    length: float
    width: float
    height: float

@dataclass
class BoxObstacleConfig(FreeCollisionObstacleConfig):
    """
    Configuration dataclass for box obstacle.

    Parameters:
    ------------
    geometry : GeometryConfig : Geometry of the obstacle
    """
    geometry: GeometryConfig

class BoxObstacle(FreeCollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(BoxObstacleConfig)
        super().__init__(schema, **kwargs)

    def length(self):
        return self._config.geometry.length

    def width(self):
        return self._config.geometry.width

    def height(self):
        return self._config.geometry.height

    def add_to_bullet(self, pybullet) -> int:
        """
        Adds object to pybullet environment.
        """
        pybullet.setAdditionalSearchPath(
                os.path.dirname(os.path.realpath(__file__)))

        collision_shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,
                halfExtents=[self.length()/2, self.width()/2, self.height()/2])

        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="box.obj",
            rgbaColor=self.color(),
            meshScale=[self.length()/2, self.width()/2, self.height()/2]
        )

        if self.dimension() == 3:
            base_position = self.position()
        else:
            raise DimensionNotSuitableForEnv(
                    "Pybullet only supports three dimensional obstacles")

        base_orientation = self.orientation()

        mass = -1
        if self.movable():
            mass = self.mass()

        self._bullet_id = pybullet.createMultiBody(
              mass,
              collision_shape,
              visual_shape_id,
              base_position,
              base_orientation
              )

        return self.bullet_id()
