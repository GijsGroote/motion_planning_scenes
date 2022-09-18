import pytest

import numpy as np

from motion_planning_env.free_collision_obstacle import FreeCollisionObstacle
from motion_planning_scene_helpers.motion_planning_component import(
        ComponentIncompleteError
        )

from omegaconf import OmegaConf

def test_free_collision_obstacle():
    obst_dict = {"type": "free_col_obstacle",
            "position": [0.1, 0.2],
            "orientation": [0, 0, 0, 1],
            "movable": True,
            "mass": 13,
            "color": [0, 1, 1, 1],
             }

    schema = OmegaConf.structured({})
    free_col_obst = FreeCollisionObstacle(schema,
            name="free_col_obstacle_name",
            content_dict=obst_dict)

    assert "free_col_obstacle_name" == free_col_obst.name()
    assert "free_col_obstacle" == free_col_obst.type()
    assert [0.1, 0.2] == free_col_obst.position()
    assert 2 == free_col_obst.dimension()
    assert [0, 0, 0, 1] == free_col_obst.orientation()
    assert free_col_obst.movable()
    assert 13 == free_col_obst.mass()
    assert [0, 1, 1, 1] == free_col_obst.color()

def test_r_euler_angles():
    obst_dict = {"type": "free_col_obstacle",
            "position": [0.1, 0.2,5],
            "orientation": [0, 1, 1],
             }
    schema = OmegaConf.structured({})
    free_col_obst = FreeCollisionObstacle(
            schema,
            name="free_col_obstacle_name",
            content_dict=obst_dict)
    answer = [-0.22984884706593015,
            0.42073549240394825,
            0.42073549240394825,
            0.7701511529340699]
    assert answer == free_col_obst.orientation()
