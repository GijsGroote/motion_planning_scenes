import pytest
import time
import os
import sys

import pybullet as p
import pybullet_data
from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.urdf_obstacle import UrdfObstacle
from motion_planning_env.dynamic_sphere_obstacle import DynamicSphereObstacle

no_gui = True

@pytest.fixture
def bullet():
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    #p.setAdditionalSearchPath(os.path.dirname(__file__) + "/models")
    return p

@pytest.fixture
def bullet_gui():
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    #p.setAdditionalSearchPath(os.path.dirname(__file__) + "/models")
    return p

def test_sphere_obstacle(bullet):
    obst_dict = {"type": "sphere", "position": [0.1, 0.2, 0.4], "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simpleSphere", content_dict=obst_dict)
    sphere_obst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        sphere_obst.update_bullet_position(bullet, t=i/10)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_sphere_obstacle_gui(bullet_gui):
    obst_dict = {"type": "sphere", "position": [0.1, 0.2, 0.4], "geometry": {"radius": 0.2}}
    sphere_obst = SphereObstacle(name="simpleSphere", content_dict=obst_dict)
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
    sphere_obst = UrdfObstacle(name="simpleUrdf", content_dict=obst_dict)
    sphere_obst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        sphere_obst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_urdfObstacle_gui(bullet_gui):
    obst_dict = {
        "type": "sphere",
        "position": [0.1, 0.2, 0.4],
        "geometry": {"urdf": "teddy_large.urdf"},
    }
    sphereObst = UrdfObstacle(name="simpleUrdf", content_dict=obst_dict)
    sphereObst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        sphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_circleObstacle_gui(bullet_gui):
    obst_dict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    dynamicSphereObst.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_circleObstacle(bullet):
    obst_dict = {
        "type": "sphere",
        "geometry": {"trajectory": ["0.1 * t", "0.2 * t"], "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    dynamicSphereObst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

def test_splineObstacle(bullet):
    splineDict = {"degree": 2, "controlPoints": [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], "duration": 10}
    obst_dict = {
        "type": "splineSphere",
        "geometry": {"trajectory": splineDict, "radius": 0.2},
    }
    dynamicSphereObst = DynamicSphereObstacle(
        name="dynamicSphere", content_dict=obst_dict
    )
    dynamicSphereObst.add_to_bullet(bullet)
    for i in range(100):
        bullet.stepSimulation()
        dynamicSphereObst.update_bullet_position(bullet, t=i/10)
        time.sleep(1/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_splineObstacle_gui(bullet_gui):
    spline_dict = {"degree": 2, "controlPoints": [[1.0, 0.0], [2.0, 0.0],[2.0, 1.0]], "duration": 10}
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
