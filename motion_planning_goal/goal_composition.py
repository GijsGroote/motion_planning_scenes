from motion_planning_scene_helpers.motion_planning_component import (
    MotionPlanningComponent,
)
from motion_planning_goal.subgoal import Subgoal
from motion_planning_goal.subgoal_creator import SubgoalCreator
from motion_planning_goal.static_joint_space_subgoal import (
    JointSpaceGoalsNotSupportedError,
)

import yaml
from omegaconf import OmegaConf


class MultiplePrimeGoalsError(Exception):
    pass


class GoalComposition(MotionPlanningComponent):
    def __init__(self, **kwargs):
        # Should here not be super.__init__(self, kwargs)?? 
        if "content_dict" in kwargs and "name" in kwargs:
            self._content_dict = kwargs.get("content_dict")
            self._name = kwargs.get("name")
        elif "file_name" in kwargs:
            with open(kwargs.get("file_name"), "r", encoding="UTF-8") as stream:
                self._content_dict = yaml.safe_load(stream)
            self._name = self._content_dict["name"]
            del self._content_dict["name"]
        self._config = OmegaConf.create(self._content_dict)
        self._primary_goal_index = -1
        self._subgoals = []
        self._subgoal_creator = SubgoalCreator()
        self.parse_subgoals()

    def parse_subgoals(self):
        for subgoal_name in self._config.keys():
            subgoal_type = self._config[subgoal_name].type
            subgoal_dict = self._config[subgoal_name]
            subgoal = self._subgoal_creator.create_subgoal(
                subgoal_type, subgoal_name, subgoal_dict
            )
            if subgoal.is_primary_goal():
                if self._primary_goal_index >= 0:
                    raise MultiplePrimeGoalsError(
                        "There are multiple prime goals. Using first prime goal"
                    )
                else:
                    self._primary_goal_index = len(self._subgoals)
            self._subgoals.append(subgoal)

    def primary_goal(self):
        return self.get_goal_by_index(self._primary_goal_index)

    def subgoals(self):
        return self._subgoals

    def get_goal_by_name(self, name) -> Subgoal:
        for subgoal in self._subgoals:
            if subgoal.name() == name:
                return subgoal

    def get_goal_by_index(self, index):
        return self._subgoals[index]

    def evaluate(self):
        evals = []
        for subgoal in self._subgoals:
            evals += subgoal.evaluate()
        return evals

    def dict(self):
        composition_dict = {}
        for subgoal in self._subgoals:
            composition_dict[subgoal.name()] = subgoal.dict()
        return composition_dict

    def shuffle(self):
        for subgoal in self._subgoals:
            subgoal.shuffle()

    def render_gym(self, viewer, rendering):
        for subgoal in self._subgoals:
            try:
                subgoal.render_gym(viewer, rendering)
            except JointSpaceGoalsNotSupportedError as _:
                print("Skipping visualization of joint space goal.")

    def add_to_bullet(self, pybullet):
        for subgoal in self._subgoals:
            try:
                subgoal.add_to_bullet(
                    pybullet, position=self.primary_goal().position()
                )
            except JointSpaceGoalsNotSupportedError as _:
                print("Skipping visualization of joint space goal.")

    def update_bullet_position(self, pybullet, **kwargs):
        self.primary_goal().update_bullet_position(pybullet, **kwargs)
