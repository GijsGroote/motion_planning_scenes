from MotionPlanningEnv.collisionObstacle import CollisionObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError

class SphereObstacleMissmatchDimensionError(Exception):
    pass

class SphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['position', 'radius']
        self.checkCompleteness()
        self.checkDimensionality()
        self.checkGeometryCompleteness()

    def checkDimensionality(self):
        if self.dim() != len(self.position()):
            raise SphereObstacleMissmatchDimensionError(
                "Dimension mismatch between position array and dimension"
            )

    def checkGeometryCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._geometry_keys:
            if key not in self.geometry():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise ComponentIncompleteError("Missing keys in geometry: %s" % missingKeys[:-2])

    def position(self, **kwargs):
        return self.geometry()['position']

    def radius(self):
        return self.geometry()['radius']

    def toDict(self):
        return self._contentDict

    def renderGym(self, viewer):
        from gym.envs.classic_control import rendering
        x = self.position()
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.radius())
        joint.add_attr(tf)