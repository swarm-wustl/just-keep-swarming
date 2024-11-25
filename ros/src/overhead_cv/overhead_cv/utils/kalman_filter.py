from math import cos, sin

import numpy as np

H = np.array(
    [
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]
)


# pylint: disable=too-many-arguments,too-many-locals
def kalman_filter(x_prev, u, z, P_prev, dt=1, alpha=0.001, beta=0.01):
    F = np.array(
        [
            [1, 0, dt * cos(x_prev[3]), 0, 0],
            [0, 1, dt * sin(x_prev[3]), 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, dt],
            [0, 0, 0, 0, 1],
        ]
    )

    B = np.array(
        [
            [0, 0],
            [0, 0],
            [dt, 0],
            [0, 0],
            [0, dt],
        ]
    )

    x_prior = F @ x_prev + B @ u

    Q = np.eye(5) * alpha
    R = np.eye(3) * beta

    P_prior = F @ P_prev @ F.T + Q

    S = H @ P_prior @ H.T + R
    K = P_prior @ H.T @ np.linalg.inv(S)
    P = (np.eye(5) - K @ H) @ P_prior

    x = x_prior + K @ (z - H @ x_prior)

    return x, P