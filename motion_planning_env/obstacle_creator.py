from motion_planning_env.sphere_obstacle import SphereObstacle
from motion_planning_env.dynamic_sphere_obstacle import DynamicSphereObstacle


class UnknownObstacleType(Exception):
    pass


class ObstacleCreator(object):
    def __init__(self):
        pass

    def create_obstacle(self, obstacle_type, name, content_dict):
        if obstacle_type == "sphereObstacle":
            return SphereObstacle(name=name, content_dict=content_dict)
        elif obstacle_type == "analyticSphere":
            return DynamicSphereObstacle(name=name, content_dict=content_dict)
        elif obstacle_type == "splineSphere":
            return DynamicSphereObstacle(name=name, content_dict=content_dict)
        else:
            raise UnknownObstacleType(
                f"ObstacleType {obstacle_type} is not know"
            )
