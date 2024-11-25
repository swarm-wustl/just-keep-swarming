import numpy as np

from overhead_cv.utils.hungarian import hungarian
from overhead_cv.utils.kalman_filter import H
from overhead_cv.utils.robot_estimator import RobotStateEstimator


# pylint: disable=too-few-public-methods
class MultiRobotStateEstimator:
    def __init__(self, q=0.09, r=0.005):
        self.estimators = {}
        self.q = q
        self.r = r
        self.estimator_id = 0

    def update_estimate(self, id2u, zs, dt=1):
        if len(zs) > len(self.estimators):
            for _ in range(len(zs) - len(self.estimators)):
                self.estimators[self.estimator_id] = RobotStateEstimator(
                    q=self.q, r=self.r
                )
                self.estimator_id += 1

        # TODO(sebtheiler): use new filtered values instead of raw observations # pylint: disable=fixme
        ids = self.estimators.keys()
        zs_prev = list(map(lambda e: H @ e.x, self.estimators))
        zs_cur = zs + [None] * (len(zs_prev) - len(zs))
        assignments = hungarian(
            zs_prev,
            zs_cur,
            ids,
        )

        for assignment in assignments:
            z = zs_cur[assignment]
            if z is None:
                continue

            u = id2u.get(assignment, np.zeros(2))

            self.estimators[assignment].update_estimate(u, z, dt)

        return assignments
