from MotionPlanningEnv.collisionObstacle import CollisionObstacleConfig, CollisionObstacle
from dataclasses import dataclass, field, KW_ONLY
from typing import List
import numpy as np
from omegaconf import OmegaConf

@dataclass
class FreeCollisionObstacleConfig(CollisionObstacleConfig):
    """
    Configuration dataclass for free collision obstacle.
    Parameters:
    ------------
    position: List: [x,y,z] Position of the obstacle
    orientation: List: [x,y,z,w] Quaternion orientation of the obstacle
        [yaw, pitch, roll] euler angles are converted to quaternion orientation
    movable : bool : Flag indicating whether an obstacle is movable
    mass: float : mass of the object, only used if movable set to true
    color : List : [r,g,b,a] where r,g,b and a are floats between 0 and 1
    """
    position: List[float]
    _: KW_ONLY
    orientation: List[float] = field(default_factory=list)
    movable: bool = True
    mass: float = 1
    color: List[float] = field(default_factory=list)

class FreeCollisionObstacle(CollisionObstacle):
    """
    Free collision obstacle containing functionality and
    checks for the arguents of the configuration.

    """
    def __init__(self, schema, **kwargs):
        schema = OmegaConf.merge(FreeCollisionObstacleConfig, schema)
        super().__init__(schema, **kwargs)
        self.check_input()

    def check_input(self):
        self.check_orientation()
        self.check_color()
        self.check_mass()

    def check_orientation(self):
        if "orientation" in self._content_dict:
            if len(self._content_dict["orientation"]) == 3:

                # convert euler to quaternion
                roll = self._content_dict["orientation"][0]
                pitch = self._content_dict["orientation"][1]
                yaw = self._content_dict["orientation"][2]

                qx = float(np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)\
                - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
                qy = float(np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)\
                + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
                qz = float(np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)\
                - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
                qw = float(np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)\
                + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))

                self._config.orientation = [qx, qy, qz, qw]

            elif len(self._content_dict["orientation"]) != 4:
                raise ValueError(f"""
                        incorrect orientation shape: {len(
                        self._content_dict["orientation"])}, which should be (4,)
                        """)

    def check_color(self):
        if "color" in self._content_dict:
            if len(self._content_dict["color"]) != 4:
                raise ValueError(f"""
                        incorrect color shape: {len(
                        self._content_dict["color"])}, which should be (4,)
                        """)

    def check_mass(self):
        if "mass" in self._content_dict:
            if self._content_dict["mass"] <= 0:
                raise ValueError(f"""
                        negative mass: 
                        {self._content_dict["mass"]}, which should positive
                        """)

    def orientation(self):
        return self._config.orientation

    def movable(self):
        return self._config.movable

    def mass(self):
        return self._config.mass

    def color(self):
        return self._config.color

    def position(self):
        return self._config.position

    def dimension(self):
        return len(self.position())

