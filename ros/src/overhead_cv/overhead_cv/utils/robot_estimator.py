import time

import numpy as np

from overhead_cv.utils.filtering_types import Command, Measurement, State
from overhead_cv.utils.kalman_filter import kalman_filter


# pylint: disable=too-few-public-methods
class RobotStateEstimator:
    def __init__(self, x=State(), q=0.09, r=0.005):
        self.x = x
        self.P = np.eye(5)
        self.q = q
        self.r = r
        self.num_estimates_received = 0
        self.last_estimate_received_at = 0

    def update_estimate(self, u: Command, z: Measurement, dt=1.0, save=True):
        if not self.num_estimates_received:
            self.x = State(z.x, z.y)

        x, P = kalman_filter(
            self.x.to_np(), u.to_np(), z.to_np(), self.P, dt, self.q, self.r
        )
        if save:
            self.x = State.from_np(x)
            self.P = P
            self.num_estimates_received += 1
            self.last_estimate_received_at = time.time()

        return x, P
