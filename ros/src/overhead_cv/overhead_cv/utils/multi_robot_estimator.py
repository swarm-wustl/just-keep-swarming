from typing import List

import numpy as np

from overhead_cv.utils.filtering_types import Measurement
from overhead_cv.utils.hungarian import hungarian
from overhead_cv.utils.robot_estimator import RobotStateEstimator


class MultiRobotStateEstimator:
    def __init__(self, num_robots, q=0.09, r=0.005):
        self.calibrating = True
        self.num_robots = num_robots
        self.estimators = [RobotStateEstimator(q=q, r=r) for _ in range(num_robots)]
        self.q = q
        self.r = r

    def update_estimate(self, actions, Z: List[Measurement], dt=1.0):
        """
        Update estimated positions for each robot based
        on some measurements of the detected world positions
        """
        current_states = [e.x[:2] for e in self.estimators]
        new_measurements = [np.array([z.x, z.y]) for z in Z]
        assignments = hungarian(current_states, new_measurements)

        for robot_i, measurement_assignment in enumerate(assignments):
            z = Z[measurement_assignment]
            z = np.array([z.x, z.y])
            print(f"robot at {self.estimators[robot_i].x} was measured at {z}")

            u = actions.get(robot_i, np.zeros(2))
            self.estimators[robot_i].update_estimate(u, z, dt)

        return assignments

    def get_key(self, est_id):
        return self.estimators[est_id].x[1] * self.estimators[est_id].x[0]

    def assign_new_ids(self):
        # Assign ids based on row then column
        robot_unsorted_id_list = list(range(self.num_robots))
        robot_unsorted_id_list.sort(key=self.get_key)

        # Swap robot_unsorted to sorted
        swapped_robots = [self.estimators[id] for id in robot_unsorted_id_list]
        self.estimators = swapped_robots
        self.calibrating = False
