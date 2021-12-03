from MotionPlanningEnv.collisionObstacle import CollisionObstacle
from MotionPlanningSceneHelpers.motionPlanningComponent import ComponentIncompleteError, DimensionNotSuitableForEnv
from MotionPlanningSceneHelpers.analyticTrajectory import AnalyticTrajectory


class DynamicSphereObstacleMissmatchDimensionError(Exception):
    pass

class DynamicSphereObstacle(CollisionObstacle):
    def __init__(self, **kwargs):
        super().__init__( **kwargs)
        self._geometry_keys = ['trajectory', 'radius']
        self.checkCompleteness()
        self.checkDimensionality()
        self.checkGeometryCompleteness()
        self._traj = AnalyticTrajectory(self.dim(), traj=self.geometry()['trajectory'])
        self._traj.concretize()

    def checkDimensionality(self):
        if self.dim() != len(self.geometry()['trajectory']):
            raise DynamicSphereObstacleMissmatchDimensionError(
                "Dimension mismatch between trajectory array and dimension"
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
        if 't' not in kwargs:
            t = 0.0
        else:
            t = kwargs.get('t')
        return self._traj.evaluate(t)[0]

    def radius(self):
        return self.geometry()['radius']

    def toDict(self):
        return self._contentDict

    def movable(self):
        return False

    def toCSV(self, fileName, samples=100):
        import numpy as np
        import csv
        theta = np.arange(-np.pi, np.pi, step=np.pi/samples)
        x = self.position()[0] + (self.radius()-0.1) * np.cos(theta)
        y = self.position()[1] + (self.radius()-0.1) * np.sin(theta)
        with open(fileName, mode='w') as file:
            csv_writer = csv.writer(file, delimiter=',')
            for i in range(2*samples):
                csv_writer.writerow([x[i], y[i]])

    def renderGym(self, viewer, **kwargs):
        from gym.envs.classic_control import rendering
        x = self.position(t=kwargs.get('t'))
        if self.dim() != 2:
            raise DimensionNotSuitableForEnv("PlanarGym only supports two dimensional obstacles")
        tf = rendering.Transform(rotation=0, translation=(x[0], x[1]))
        joint = viewer.draw_circle(self.radius())
        joint.add_attr(tf)

    def add2Bullet(self, pybullet):
        if self.dim() != 3:
            raise DimensionNotSuitableForEnv("Pybullet only supports three dimensional obstacles")
        collisionShape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=self.radius())
        visualShapeId = -1
        basePosition = self.position()
        baseOrientation = [0, 0, 0, 1]
        mass = int(self.movable())
        visualShapeId = -1

        pybullet.createMultiBody(mass,
              collisionShape,
              visualShapeId,
              basePosition,
              baseOrientation)
