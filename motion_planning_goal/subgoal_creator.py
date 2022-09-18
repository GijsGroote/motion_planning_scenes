from motion_planning_goal.static_subgoal import StaticSubgoal
from motion_planning_goal.static_joint_space_subgoal import StaticJointSpaceSubgoal
from motion_planning_goal.dynamic_subgoal import DynamicSubgoal


class UnknownSubgoalType(Exception):
    pass


class SubgoalCreator(object):
    def __init__(self):
        pass

    def create_subgoal(self, sub_goal_type, name, content_dict):
        if sub_goal_type == "static_subgoal":
            return StaticSubgoal(name=name, content_dict=content_dict)
        elif sub_goal_type in ("analytic_subgoal", "spline_subgoal"):
            return DynamicSubgoal(name=name, content_dict=content_dict)
        elif sub_goal_type == "static_joint_space_subgoal":
            return StaticJointSpaceSubgoal(name=name, content_dict=content_dict)
        else:
            raise UnknownSubgoalType(
                f"SubgoalType {sub_goal_type} is not known"
            )
