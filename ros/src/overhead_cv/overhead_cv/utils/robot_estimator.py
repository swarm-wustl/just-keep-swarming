import time

import numpy as np

from overhead_cv.utils.extended_kalman_filter import extended_kalman_filter
from overhead_cv.utils.filtering_types import LEN_STATE_VEC, Command, Measurement, State
from overhead_cv.utils.low_pass_filter import low_pass_filter
from overhead_cv.utils.normalize_angle import normalize_angle


# pylint: disable=too-few-public-methods,too-many-instance-attributes
class RobotStateEstimator:
    def __init__(self, x=State(), q=0.09, r=0.005):
        self.x = x
        self.P = np.eye(LEN_STATE_VEC)
        self.q = q
        self.r = r
        self.num_estimates_received = 0
        self.last_estimate_received_at = 0
        self.dist_threshold = 0.20  # meters
        self.ang_threshold = 90 * np.pi / 180  # rad
        self.reset_counter = 0

    def update_estimate(self, u: Command, z: Measurement, dt=1.0, save=True):
        # Reset
        if not self.num_estimates_received or self.reset_counter > 20:
            self.x = State(z.x, z.y, 0, np.cos(z.theta), np.sin(z.theta), 0)
        # Discard unreliable measurements
        if (
            math.hypot(self.x.x - z.x, self.x.y - z.y) > self.dist_threshold
            or abs(
                normalize_angle(math.atan2(self.x.sin_theta, self.x.cos_theta))
                - normalize_angle(z.theta)
            )
            > self.ang_threshold
        ) and self.num_estimates_received > 50:
            self.reset_counter += 1
            return self.x, self.P

        self.reset_counter = 0

        x, P = extended_kalman_filter(
            self.x.to_np(), u.to_np(), z.to_np(), self.P, dt, self.q, self.r
        )

        x[3] = low_pass_filter(x[3], self.x.sin_theta, alpha=0.8)
        x[4] = low_pass_filter(x[4], self.x.cos_theta, alpha=0.8)

        if save:
            self.x = State.from_np(x)
            self.P = P
            self.num_estimates_received += 1
            self.last_estimate_received_at = time.time()

        return x, P
