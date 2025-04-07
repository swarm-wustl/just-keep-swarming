from typing import Dict, List

from overhead_cv.utils.filtering_types import Command, Measurement
from overhead_cv.utils.hungarian import hungarian
from overhead_cv.utils.robot_estimator import RobotStateEstimator


class MultiRobotStateEstimator:
    def __init__(self, num_robots: int, q=0.09, r=0.005):
        self.num_robots = num_robots
        self.estimators = [RobotStateEstimator(q=q, r=r) for _ in range(num_robots)]
        self.q = q
        self.r = r

    def update_estimate(
        self, actions: Dict[int, Command], measurements: List[Measurement], dt=1.0
    ) -> List[int]:
        """
        Update estimated positions for each robot based
        on some measurements of the detected world positions
        """
        current_states = [e.x for e in self.estimators]
        assignments = hungarian(current_states, measurements)

        for robot_i, measurement_assignment in enumerate(assignments):
            z = measurements[measurement_assignment]
            u = actions.get(robot_i, Command(0, 0))
            self.estimators[robot_i].update_estimate(u, z, dt)

        return assignments

    def get_key(self, est_id: int) -> float:
        return self.estimators[est_id].x.x * self.estimators[est_id].x.y

    def assign_new_ids(self):
        """
        After calibration, reorder the detected robot ids based on row then column
        """
        robot_unsorted_id_list = list(range(self.num_robots))
        robot_unsorted_id_list.sort(key=self.get_key)

        # Swap robot_unsorted to sorted
        swapped_robots = [self.estimators[id] for id in robot_unsorted_id_list]
        self.estimators = swapped_robots
