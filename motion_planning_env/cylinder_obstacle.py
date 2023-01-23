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
    This configuration dataclass holds information about
    the geometry of the cylinder obstacle.
    Parameters:
    ------------
    radius: float: Radius of the obstacle
    height: float: Height of the obstacle
    """
    radius: float
    height: float

@dataclass
class CylinderObstacleConfig(FreeCollisionObstacleConfig):
    """
    Configuration dataclass for cylinder obstacle.

    Parameters:
    ------------

    geometry : GeometryConfig : Geometry of the obstacle
    """
    geometry: GeometryConfig

class CylinderObstacle(FreeCollisionObstacle):
    def __init__(self, **kwargs):
        schema = OmegaConf.structured(CylinderObstacleConfig)
        super().__init__(schema, **kwargs)

    def radius(self):
        return self._config.geometry.radius

    def height(self):
        return self._config.geometry.height

    def add_to_bullet(self, pybullet) -> int:
        """
        Adds object to pybullet environment.
        """
        pybullet.setAdditionalSearchPath(
                os.path.dirname(os.path.realpath(__file__)))

        mass = -1
        if self.movable():
            mass = self.mass()

        collision_shape = pybullet.createCollisionShape(
                pybullet.GEOM_CYLINDER,
                radius=self.radius(),
                height=self.height())

        visual_shape_id = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="cylinder.obj",
            rgbaColor=self.color(),
            meshScale=[self.radius(), self.radius(), self.height()/2]
        )

        # if the obstacle requires a target ghost position

        if len(self.color()) == 0:
            color = [200, 200, 200, 0.5]
        else:
            color = [self.color()[0], self.color()[1], self.color()[2], 0.5]

        self.ghost_visual_shape = pybullet.createVisualShape(
            pybullet.GEOM_MESH,
            fileName="cylinder.obj",
            rgbaColor=color,
            meshScale=[self.radius(), self.radius(), self.height()/2]
        )

        if self.dimension() == 3:
            base_position = self.position()
        else:
            raise DimensionNotSuitableForEnv(
                    "Pybullet only supports three dimensional obstacles")

        base_orientation = self.orientation()

        self._bullet_id = pybullet.createMultiBody(
              mass,
              collision_shape,
              visual_shape_id,
              base_position,
              base_orientation
              )

        return self.bullet_id()
