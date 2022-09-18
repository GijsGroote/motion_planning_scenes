from omegaconf.errors import MissingMandatoryValue
import pytest

from motion_planning_env.cylinder_obstacle import CylinderObstacle

def test_sphere_obstacle():
    obst_dict = {"type": "cylinder",
            "position": [0.1, 0.2, 0.4],
            "geometry": {"radius": 0.2,
            "height": 1}}

    sphere_obst = CylinderObstacle(name="simpleSphere", content_dict=obst_dict)
    assert "simpleSphere" == sphere_obst.name()
    assert [0.1, 0.2, 0.4] == sphere_obst.position()
    assert 0.2 == sphere_obst.radius()
    assert 1 == sphere_obst.height()
    assert 3 == sphere_obst.dimension()

def test_error_raise_incomplete_dict():
    obst_dict = {"type": "cylinder", "position": [0.1, 0.2], "geometry": {}}
    with pytest.raises(MissingMandatoryValue):
        cylinder = CylinderObstacle(name="simpleCylinder",
                content_dict=obst_dict)
        cylinder.height()

def test_radius_height_incorrect_type():
    not_floats = [[0.5], [0.5, 0,5], None, {}]

    for nofloat in not_floats:

        obs_dict = {"type": "cylinder", "position": [0.1, 0.2, 0.5],
                "geometry": {"radius": nofloat, "height": 0.5}}
        with pytest.raises(ValueError) as _:
            CylinderObstacle(name="simpleCylinder", content_dict=obs_dict)
        obs_dict = {"type": "cylinder", "position": [0.1, 0.2, 0.5],
                "geometry": {"radius": 0.5, "height": nofloat}}
        with pytest.raises(ValueError) as _:
            CylinderObstacle(name="simpleCylinder", content_dict=obs_dict)
