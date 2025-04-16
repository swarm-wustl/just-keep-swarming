import time

import numpy as np

from overhead_cv.utils.filtering_types import Command, Measurement, State
from overhead_cv.utils.kalman_filter import kalman_filter
from overhead_cv.utils.normalize_angle import normalize_angle


def low_pass_filter(new_angle, prev_filtered, alpha=0.15):
    return alpha * new_angle + (1 - alpha) * prev_filtered


# pylint: disable=too-few-public-methods,too-many-instance-attributes
class RobotStateEstimator:
    def __init__(self, x=State(), q=0.09, r=0.005):
        self.x = x
        self.P = np.eye(5)
        self.q = q
        self.r = r
        self.num_estimates_received = 0
        self.last_estimate_received_at = 0
        self.dist_threshold = 0.20  # meters
        self.ang_threshold = 90 * np.pi / 180  # rad
        self.reset_counter = 0

    def update_estimate(self, u: Command, z: Measurement, dt=1.0, save=True):
        if not self.num_estimates_received or self.reset_counter > 20:
            self.x = State(z.x, z.y, 0, z.theta)
            self.P = np.eye(5)
            print("reset!")

        # Discard unreliable measurements
        # if (
        #     math.hypot(self.x.x - z.x, self.x.y - z.y) > self.dist_threshold
        #     or abs(normalize_angle(self.x.orientation) - normalize_angle(z.theta))
        #     > self.ang_threshold
        # ) and self.num_estimates_received > 50:
        #     print(math.hypot(self.x.x - z.x, self.x.y - z.y), self.dist_threshold)
        #     print(abs(normalize_angle(self.x.orientation) - normalize_angle(z.theta)), self.ang_threshold)
        #     self.reset_counter += 1
        #     return self.x, self.P

        self.reset_counter = 0

        x, P = kalman_filter(
            self.x.to_np(), u.to_np(), z.to_np(), self.P, dt, self.q, self.r
        )
        x[2] = 0
        x[4] = 0
        x[3] = normalize_angle(low_pass_filter(x[3], self.x.orientation, alpha=0.8))

        print(x)
        print(P)
        # x = np.array([
        #     low_pass_filter(z.x, self.x.x, 0.8),
        #     low_pass_filter(z.y, self.x.y, 0.8),
        #     0,
        #     low_pass_filter(z.theta, self.x.orientation, 0.8),
        #     0,
        # ])
        # P = np.eye(5)

        if save:
            self.x = State.from_np(x)
            self.P = P
            self.num_estimates_received += 1
            self.last_estimate_received_at = time.time()

        return x, P
