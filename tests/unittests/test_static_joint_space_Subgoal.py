import pytest

from motion_planning_goal.static_joint_space_subgoal import StaticJointSpaceSubgoal
from motion_planning_goal.subgoal import SubgoalMissmatchDimensionError
from motion_planning_scene_helpers.motion_planning_component import ComponentIncompleteError

from omegaconf.errors import MissingMandatoryValue

@pytest.fixture
def simpleGoalDict():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "desired_position": [0.01, 0.2, 0.5],
        "epsilon": 0.2,
        "type": "staticJointSpaceSubgoal",
        "indices": [0, 1, 2], 
    }
    return goalDict


def test_staticSubgoal(simpleGoalDict):
    simpleStaticSubgoal = StaticJointSpaceSubgoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    assert "simple_static_subGoal" == simpleStaticSubgoal.name()
    assert [0.01, 0.2, 0.5] == simpleStaticSubgoal.position()
    assert 0.2 == simpleStaticSubgoal.epsilon()
    assert "staticJointSpaceSubgoal" == simpleStaticSubgoal.type()

def test_shuffleGoal(simpleGoalDict):
    simpleStaticSubgoal = StaticJointSpaceSubgoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubgoal.shuffle()
    assert [0.01, 0.2, 0.5] != simpleStaticSubgoal.position()
    assert simpleStaticSubgoal.position()[0] >= -1
    assert simpleStaticSubgoal.position()[0] <= 1
    assert simpleStaticSubgoal.position()[1] >= -1
    assert simpleStaticSubgoal.position()[1] <= 1
    # add limits to goalDict
    simpleGoalDict['low'] = [-2, -2, -2]
    simpleGoalDict['high'] = [-1, -1, 0]
    simpleStaticSubgoal = StaticJointSpaceSubgoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubgoal.shuffle()
    assert [0.01, 0.2, 0.5] != simpleStaticSubgoal.position()
    assert simpleStaticSubgoal.position()[0] >= -2
    assert simpleStaticSubgoal.position()[0] <= -1
    assert simpleStaticSubgoal.position()[1] >= -2
    assert simpleStaticSubgoal.position()[1] <= -1
    assert simpleStaticSubgoal.position()[2] >= -2
    assert simpleStaticSubgoal.position()[2] <= 0

def test_saving_sub_goal(simpleGoalDict):
    simpleStaticSubgoal = StaticJointSpaceSubgoal(name="simple_static_subGoal", content_dict=simpleGoalDict)
    simpleStaticSubgoal.shuffle()
    goal_dict_after =simpleStaticSubgoal.dict()
    assert isinstance(goal_dict_after, dict)
    assert goal_dict_after['desired_position'][0] != 0.01


def test_errorRaiseIncompleteDict():
    goalDict = {
        "is_primary_goal": True,
        "indices": [0, 1],
        "desired_position": [0.01, 0.2],
    }
    with pytest.raises(MissingMandatoryValue) as e_info:
        static_sub_goal = StaticJointSpaceSubgoal(name="example_static_subGoal", content_dict=goalDict)
        weight = static_sub_goal.weight()


def test_errorRaiseMissmatichDimension():
    goalDict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "epsilon": 0.2,
        "indices": [0],
        "desired_position": [0.01, 0.2],
        "type": "staticJointSpaceSubgoal",
    }
    with pytest.raises(SubgoalMissmatchDimensionError) as e_info:
        StaticJointSpaceSubgoal(name="example_static_subGoal", content_dict=goalDict)
