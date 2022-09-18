from motion_planning_env.dynamic_sphere_obstacle import DynamicSphereObstacle


def test_circle_obstacle():
    obst_dict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamic_sphere_obst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    assert "dynamicSphere" == dynamic_sphere_obst.name()
    assert 0.2 == dynamic_sphere_obst.radius()
    pos_t_00 = dynamic_sphere_obst.position(t=0.0)
    assert pos_t_00[0] == 0.0
    assert pos_t_00[1] == 0.0
    pos_t_12 = dynamic_sphere_obst.position(t=1.2)
    assert pos_t_12[0] == 0.1 * 1.2
    assert pos_t_12[1] == 0.2 * 1.2


def test_spline_obstace():
    spline_dict = {"degree": 2,
            "controlPoints": [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]],
            "duration": 10}
    obst_dict = {
        "type": "splineSphere",
        "geometry": {"trajectory": spline_dict, "radius": 0.2},
    }
    dynamic_sphere_obst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    assert "dynamicSphere" == dynamic_sphere_obst.name()
    assert 0.2 == dynamic_sphere_obst.radius()
