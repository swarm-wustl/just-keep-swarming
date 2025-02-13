import time
import numpy as np
from overhead_cv.utils.kalman_filter import kalman_filter


# pylint: disable=too-few-public-methods
class RobotStateEstimator:
    def __init__(self, x=np.zeros(5), q=0.09, r=0.005):
        self.x = x
        self.P = np.eye(5)
        self.q = q
        self.r = r
        self.num_estimates_received = 0
        self.last_estimate_received_at = 0

    def update_estimate(self, u, z, dt=1, save=True):
        x, P = kalman_filter(self.x, u, z, self.P, dt, self.q, self.r)
        if save:
            self.x = x
            self.P = P
            self.num_estimates_received += 1
            self.last_estimate_received_at = time.time()

        return x, P
