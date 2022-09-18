import pytest
from motion_planning_goal.static_joint_space_subgoal import StaticJointSpaceSubgoal
from motion_planning_goal.subgoal_creator import SubgoalCreator, UnknownSubgoalType


def test_subgoal_creator():
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
    subgoal_creator = SubgoalCreator()
    subgoal0 = subgoal_creator.create_subgoal(
        "static_subgoal", "subgoal0", goal_dict
    )
    assert subgoal0.indices() == [0, 1]


def test_subgoal_creator_joint_space_goal():
    goal_dict = {
        "weight": 5.0,
        "is_primary_goal": True,
        "indices": [0, 1],
        "desired_position": [0.01, 0.2],
        "epsilon": 0.2,
        "type": "static_joint_space_subgoal",
    }
    subgoal_creator = SubgoalCreator()
    subgoal_0 = subgoal_creator.create_subgoal(
        "static_joint_space_subgoal", "subgoal0", goal_dict
    )
    assert subgoal_0.type() == "static_joint_space_subgoal"
    assert isinstance(subgoal_0, StaticJointSpaceSubgoal)


def test_unknown_subgoal_type_error():
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
    subgoal_creator = SubgoalCreator()
    with pytest.raises(UnknownSubgoalType):
        subgoal_creator.create_subgoal("stateSubgoal", "subgoal0", goal_dict)
