import pytest

from motion_planning_goal.dynamic_subgoal import DynamicSubgoal


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


def test_dynamic_subgoal(dynamic_goal_dict):
    dynamic_subgoal = DynamicSubgoal(name="simple_dynamic_subgoal", content_dict=dynamic_goal_dict)
    assert "simple_dynamic_subgoal" == dynamic_subgoal.name()
    assert 0.2 == dynamic_subgoal.epsilon()
    assert [0.01, 0.2] == dynamic_subgoal.position(t=0).tolist()
    assert [1.01, 0.2] == dynamic_subgoal.position(t=1).tolist()

def test_dynamic_spline_subgoal(dynamic_spline_goal_dict):
    dynamic_subgoal = DynamicSubgoal(name="simple_dynamic_subgoal", content_dict=dynamic_spline_goal_dict)
    assert "simple_dynamic_subgoal" == dynamic_subgoal.name()
    assert 0.2 == dynamic_subgoal.epsilon()

