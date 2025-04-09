import time

import numpy as np

from overhead_cv.utils.filtering_types import Command, Measurement, State
from overhead_cv.utils.kalman_filter import kalman_filter


def low_pass_filter(new_angle, prev_filtered, alpha=0.15):
    return alpha * new_angle + (1 - alpha) * prev_filtered


# pylint: disable=too-few-public-methods
class RobotStateEstimator:
    def __init__(self, x=State(), q=0.09, r=0.005):
        self.x = x
        self.P = np.eye(5)
        self.q = q
        self.r = r
        self.num_estimates_received = 0
        self.last_estimate_received_at = 0
        self.dist_threshold = 0.20  # meters
        self.ang_threshold = 40 * np.pi / 180  # rad

    def update_estimate(self, u: Command, z: Measurement, dt=1.0, save=True):
        if not self.num_estimates_received:  # or self.reset_counter > 20:
            self.x = State(z.x, z.y, 0, z.theta)

        # if z.theta == 0:
        #     return self.x, self.P
        # if (
        #     math.hypot(self.x.x - z.x, self.x.y - z.y) > self.DIST_THRESHOLD
        #     or abs(self.x.orientation - z.theta) > self.ANG_THRESHOLD
        # ) and self.num_estimates_received > 50:
        #     print("unreliable")
        #     print(math.hypot(self.x.x - z.x, self.x.y - z.y), self.DIST_THRESHOLD)
        #     print(abs(self.x.orientation - z.theta), self.ANG_THRESHOLD)
        #     self.reset_counter += 1
        #     # unreliable measurement
        #     return self.x, self.P
        # else:
        #     self.reset_counter = 0

        x, P = kalman_filter(
            self.x.to_np(), u.to_np(), z.to_np(), self.P, dt, self.q, self.r
        )
        x[3] = low_pass_filter(z.theta, self.x.orientation)

        if save:
            self.x = State.from_np(x)
            self.P = P
            self.num_estimates_received += 1
            self.last_estimate_received_at = time.time()

        return x, P
