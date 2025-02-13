import time
import numpy as np

from overhead_cv.utils.hungarian import hungarian
from overhead_cv.utils.kalman_filter import H
from overhead_cv.utils.robot_estimator import RobotStateEstimator


# pylint: disable=too-few-public-methods
class MultiRobotStateEstimator:
    def __init__(self, q=0.09, r=0.005):

        self.calibrating = True
        self.estimators = {}
        self.q = q
        self.r = r
        self.estimator_id = 0

    def update_estimate(self, id2u, zs, dt=1):

        if len(zs) > len(self.estimators):
            if self.calibrating:
                for _ in range(len(zs) - len(self.estimators)):
                    self.estimators[self.estimator_id] = RobotStateEstimator(
                        q=self.q, r=self.r
                    )
                    self.estimator_id += 1
            else:
                # TODO(seb and joe): handle overflowing when not calibrating # pylint: disable=fixme
                pass

        # TODO(sebtheiler): use new filtered values instead of raw observations # pylint: disable=fixme
        ids = list(self.estimators.keys())
        zs_prev = list(map(lambda e: H @ e.x, self.estimators.values()))
        zs_cur = zs + [None] * (len(zs_prev) - len(zs))
        assignments = hungarian(
            zs_prev,
            zs_cur,
            ids,
        )

        print(assignments)
        for i, assignment in enumerate(assignments):
            z = zs_cur[i]
            if z is None:
                continue

            u = id2u.get(assignment, np.zeros(2))
            self.estimators[assignment].update_estimate(u, z, dt)

        self.remove_unseen()

        return assignments

    def remove_unseen(self):
        estimator_ids_to_remove = []
        for estimator_id, estimator in self.estimators.items():
            now = time.time()
            if (now - estimator.last_estimate_received_at > 0.5) or (
                estimator.num_estimates_received < 20
                and now - estimator.last_estimate_received_at > 0.01
            ):  # TODO(sebtheiler): make param # pylint: disable=fixme
                estimator_ids_to_remove.append(estimator_id)

        for estimator_id in estimator_ids_to_remove:
            print("robot no longer detected!")
            del self.estimators[estimator_id]

    def get_key(self, est_id):
        return self.estimators[est_id].x[1] * self.estimators[est_id].x[0]

    def assign_new_ids(self):
        # we will be assigning ids based on row first then column
        robot_unsorted_list = np.arange(0, self.estimator_id - 1)

        robot_unsorted_list.sort(key=self.get_key)

        # swap robot_unsorted to sorted

        replacement_robots = {}

        for i, val in enumerate(robot_unsorted_list):
            replacement_robots[val] = self.estimators[i]
        self.estimators = replacement_robots
        self.calibrating = False
