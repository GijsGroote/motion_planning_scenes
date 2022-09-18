import pytest

from motion_planning_goal.static_subgoal import StaticSubgoal
from motion_planning_goal.subgoal import SubgoalMissmatchDimensionError

from omegaconf.errors import MissingMandatoryValue


@pytest.fixture
def simple_goal_dict():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "static_subgoal",
    }
    return goal_dict


def test_static_subgoal(simple_goal_dict):
    simple_static_subgoal = StaticSubgoal(
            name="simple_static_subgoal",
            content_dict=simple_goal_dict)
    assert "simple_static_subgoal" == simple_static_subgoal.name()
    assert [0.01, 0.2] == simple_static_subgoal.position()
    assert 0.2 == simple_static_subgoal.epsilon()


def test_shuffle_goal(simple_goal_dict):
    simple_static_subgoal = StaticSubgoal(
            name="simple_static_subGoal",
            content_dict=simple_goal_dict)
    simple_static_subgoal.shuffle()
    assert [0.01, 0.2] != simple_static_subgoal.position()
    assert simple_static_subgoal.position()[0] >= -1
    assert simple_static_subgoal.position()[0] <= 1
    assert simple_static_subgoal.position()[1] >= -1
    assert simple_static_subgoal.position()[1] <= 1
    # add limits to goalDict
    simple_goal_dict["low"] = [-2, -2]
    simple_goal_dict["high"] = [-1, -1]
    simple_static_subgoal = StaticSubgoal(
            name="simple_static_subgoal",
            content_dict=simple_goal_dict)
    simple_static_subgoal.shuffle()
    assert [0.01, 0.2] != simple_static_subgoal.position()
    assert simple_static_subgoal.position()[0] >= -2
    assert simple_static_subgoal.position()[0] <= -1
    assert simple_static_subgoal.position()[1] >= -2
    assert simple_static_subgoal.position()[1] <= -1

def test_saving_sub_goal(simple_goal_dict):
    simple_static_subgoal = StaticSubgoal(
            name="simple_static_subGoal",
            content_dict=simple_goal_dict)
    simple_static_subgoal.shuffle()
    goal_dict_after =simple_static_subgoal.dict()
    assert isinstance(goal_dict_after, dict)
    assert goal_dict_after["desired_position"][0] != 0.01


def test_error_raise_incomplete_dict():
    goal_dict = {
        "weight": 5.0,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
    }
    with pytest.raises(MissingMandatoryValue) as e_info:
        static_sub_goal = StaticSubgoal(
            name="example_static_subgoal",
            content_dict=goal_dict)
        static_sub_goal.is_primary_goal()


def test_error_raise_missmatich_dimension():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "epsilon": 0.2,
        "indices": [0],
        "parent_link": 0,
        "child_link": 3,
        "desired_position": [0.01, 0.2],
        "type": "static_subgoal",
    }

    with pytest.raises(SubgoalMissmatchDimensionError) as e_info:
        StaticSubgoal(name="example_static_subGoal", content_dict=goal_dict)
