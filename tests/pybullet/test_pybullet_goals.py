import pytest
import time

import pybullet as p
import pybullet_data

from motion_planning_goal.static_subgoal import StaticSubgoal
from motion_planning_goal.dynamic_subgoal import DynamicSubgoal
from motion_planning_goal.static_joint_space_subgoal import (
    StaticJointSpaceSubgoal,
    JointSpaceGoalsNotSupportedError
)

no_gui = True

@pytest.fixture
def simple_goal_dict():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1, 2],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2, 1.0],
        "epsilon": 0.2,
        "type": "static_subgoal",
    }
    return goal_dict

@pytest.fixture
def simple_joint_space_goal_dict():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1, 2],
        "desired_position": [0.01, 0.2, 1.0],
        "epsilon": 0.2,
        "type": "static_joint_space_subgoal",
    }
    return goal_dict

@pytest.fixture
def dynamic_goal_dict():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "trajectory": ["0.01 + t*1", "0.2"],
        "epsilon": 0.2,
        "type": "analytic_subgoal",
    }
    return goal_dict

@pytest.fixture
def dynamic_spline_goal_dict():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "trajectory": {'degree': 2, 'controlPoints': [[0.1, 0.0], [1.0, 1.0], [1.0, 2.0]], 'duration': 10},
        "epsilon": 0.2,
        "type": "spline_subgoal",
    }
    return goal_dict 


@pytest.fixture
def bullet():
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    return p

@pytest.fixture
def bullet_gui():
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    return p

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_add_static_subgoal_gui(simple_goal_dict, bullet_gui):
    static_subgoal = StaticSubgoal(name="simple_static_subgoal", content_dict=simple_goal_dict)
    static_subgoal.add_to_bullet(bullet_gui)
    for _ in range(10):
        bullet_gui.stepSimulation()
        time.sleep(0.1)
    bullet_gui.disconnect()

def test_add_static_subgoal(simple_goal_dict, bullet):
    static_subgoal = StaticSubgoal(name="simple_static_subgoal", content_dict=simple_goal_dict)
    static_subgoal.add_to_bullet(bullet)
    for _ in range(10):
        bullet.stepSimulation()
    bullet.disconnect()

def test_add_joint_space_goal(simple_joint_space_goal_dict, bullet):
    static_subgoal = StaticJointSpaceSubgoal(name="simple_static_subgoal", content_dict=simple_joint_space_goal_dict)
    with pytest.raises(JointSpaceGoalsNotSupportedError):
        static_subgoal.add_to_bullet(bullet)
        for _ in range(10):
            bullet.stepSimulation()
    bullet.disconnect()


def test_dynamic_subgoal(dynamic_goal_dict, bullet):
    dynamic_subgoal = DynamicSubgoal(name="simple_dynamic_subgoal", content_dict=dynamic_goal_dict)
    dynamic_subgoal.add_to_bullet(bullet)
    for i in range(10):
        bullet.stepSimulation()
        dynamic_subgoal.update_bullet_position(bullet, t=i/100)
        time.sleep(i/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_dynamicSubgoal_gui(dynamic_goal_dict, bullet_gui):
    dynamic_subgoal = DynamicSubgoal(name="simple_dynamic_subgoal", content_dict=dynamic_goal_dict)
    dynamic_subgoal.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamic_subgoal.update_bullet_position(bullet_gui, t=i/100)
        time.sleep(1/100)
    bullet_gui.disconnect()

def test_dynamicSplineSubgoal(dynamic_spline_goal_dict, bullet):
    dynamic_subgoal = DynamicSubgoal(name="simple_dynamic_subGoal", content_dict=dynamic_spline_goal_dict)
    dynamic_subgoal.add_to_bullet(bullet)
    for i in range(10):
        bullet.stepSimulation()
        dynamic_subgoal.update_bullet_position(bullet, t=i/100)
        time.sleep(i/100)
    bullet.disconnect()

@pytest.mark.skipif(no_gui, reason="Not testing because gui is not available")
def test_dynamic_spline_subgoal_gui(dynamic_spline_goal_dict, bullet_gui):
    dynamic_subgoal = DynamicSubgoal(
            name="simple_dynamic_subgoal",
            content_dict=dynamic_spline_goal_dict)
    dynamic_subgoal.add_to_bullet(bullet_gui)
    for i in range(100):
        bullet_gui.stepSimulation()
        dynamic_subgoal.update_bullet_position(bullet_gui, t=i/10)
        time.sleep(1/100)
    bullet_gui.disconnect()

