import os

from motion_planning_env.urdf_obstacle import UrdfObstacle


def test_urdf_obstacle():
    obst_dict = {
        "type": "sphere",
        "position": [0.1, 0.2, 0.4],
        "geometry": {"urdf": "duck.urdf"},
    }
    sphere_obst = UrdfObstacle(name="simpleUrdf", content_dict=obst_dict)
    assert "simpleUrdf" == sphere_obst.name()
    assert [0.1, 0.2, 0.4] == sphere_obst.position()
    assert "duck.urdf" == sphere_obst.urdf()[-9:]


def test_yaml_load():
    yaml_file = os.path.join(os.path.dirname(__file__),
            "yaml_examples/urdf_sphere.yaml")
    sphere_obst = UrdfObstacle(file_name=yaml_file)
    assert "simple_urdf" == sphere_obst.name()
    assert [0.1, 0.2, 0.4] == sphere_obst.position()
    assert "sphere_015.urdf" in sphere_obst.urdf()
