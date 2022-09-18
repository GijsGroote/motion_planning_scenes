import pytest

from motion_planning_goal.goal_composition import GoalComposition, MultiplePrimeGoalsError


def test_goal_composition_single():
    goal_dict = {
        "subgoal0": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "static_subgoal",
        }
    }
    goal_composition = GoalComposition(
        name="example_static_subgoal", content_dict=goal_dict
    )
    subgoal_0 = goal_composition.primary_goal()
    assert "subgoal0" == subgoal_0.name()
    assert subgoal_0.indices() == [0, 1]
    assert subgoal_0.parent_link() == 0
    assert isinstance(subgoal_0.parent_link(), int)


@pytest.fixture
def multi_goal_dict():
    goal_dict = {
        "subgoal0": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "static_subgoal",
        },
        "subgoal1": {
            "weight": 5.0,
            "is_primary_goal": False,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [-0.21, 0.2],
            "epsilon": 0.2,
            "type": "static_subgoal",
        },
    }
    return goal_dict


def test_goal_composition_multi(multi_goal_dict):
    goal_composition = GoalComposition(
        name="example_static_subgoal", content_dict=multi_goal_dict
    )
    subgoal_1 = goal_composition.get_goal_by_name("subgoal1")
    assert "subgoal1" == subgoal_1.name()
    assert subgoal_1.position() == [-0.21, 0.2]
    subgoals = goal_composition.subgoals()
    assert len(subgoals) == 2


def test_shuffle_goal_composition(multi_goal_dict):
    goal_composition = GoalComposition(
        name="example_static_subgoal", content_dict=multi_goal_dict
    )
    # verification that returns are actually only pointers
    subgoal_1 = goal_composition.get_goal_by_name("subgoal1")
    assert subgoal_1.position() == [-0.21, 0.2]
    goal_composition.shuffle()
    assert goal_composition.get_goal_by_name(
            "subgoal1").position() != [-0.21, 0.2]
    assert subgoal_1.position() != [-0.21, 0.2]

def test_error_multiple_prime_goals():
    goal_dict = {
        "subgoal0": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [0.01, 0.2],
            "epsilon": 0.2,
            "type": "static_subgoal",
        },
        "subgoal1": {
            "weight": 5.0,
            "is_primary_goal": True,
            "indices": [0, 1],
            "parent_link": 0,
            "child_link": 3,
            "desired_position": [-0.21, 0.2],
            "epsilon": 0.2,
            "type": "static_subgoal",
        },
    }
    with pytest.raises(MultiplePrimeGoalsError) as e_info:
        GoalComposition(name="example_static_subgoal", content_dict=goal_dict)
