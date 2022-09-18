import pytest
import time

import pybullet as p
import pybullet_data
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.dynamic_sphere_obstacle import DynamicSphereObstacle

no_gui = True

@pytest.fixture
def bullet():
    p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    p.loadURDF("plane.urdf")
    return p

@pytest.fixture
def bullet_gui():
    p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    p.loadURDF("plane.urdf")
    return p

def test_sphere_obstacle(bullet):
    obst_dict = {"type": "sphere",
            "position": [0.1, 0.2, 0.4],
            "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simpleSphere", content_dict=obst_dict)
    sphere_obst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        sphere_obst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_sphere_obstacle_gui(bullet_gui):
    obst_dict = {"type": "sphere",
        "position": [0.1, 0.2, 0.4],
        "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simple_sphere", content_dict=obst_dict)
    sphere_obst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphere_obst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_urdf_obstacle(bullet):
    obst_dict = {
        "type": "sphere",
        "position": [0.1, 0.2, 0.4],
        "geometry": {"urdf": "teddy_large.urdf"},
    }
    sphere_obst = UrdfObstacle(name="simple_urdf", content_dict=obst_dict)
    sphere_obst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        sphere_obst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_urdf_obstacle_gui(bullet_gui):
    obst_dict = {
        "type": "sphere",
        "position": [0.1, 0.2, 0.4],
        "geometry": {"urdf": "teddy_large.urdf"},
    }
    sphere_obst = UrdfObstacle(name="simple_urdf", content_dict=obst_dict)
    sphere_obst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphere_obst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_circle_obstacle_gui(bullet_gui):
    obst_dict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamic_sphere_obst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    dynamic_sphere_obst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamic_sphere_obst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_circle_obstacle(bullet):
    obst_dict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamic_sphere_obst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    dynamic_sphere_obst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        dynamic_sphere_obst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

def test_spline_obstacle(bullet):
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
    dynamic_sphere_obst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        dynamic_sphere_obst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_spline_obstacle_gui(bullet_gui):
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
    dynamic_sphere_obst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamic_sphere_obst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()
