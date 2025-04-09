from math import cos, sin
from typing import Tuple

import numpy as np
from numpy.typing import NDArray

H = np.array(
    [
        [1, 0, 0, 0, 0],  # x
        [0, 1, 0, 0, 0],  # y
        [0, 0, 0, 1, 0],  # theta
    ]
)


def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


# pylint: disable=too-many-arguments,too-many-locals
def kalman_filter(
    x_prev: NDArray,
    u: NDArray,
    z: NDArray,
    P_prev: NDArray,
    dt=1.0,
    alpha=0.001,
    beta=0.01,
) -> Tuple[NDArray, NDArray]:
    z[2] = normalize_angle(z[2])
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

    # Q = np.eye(5) * alpha
    Q = np.diag([alpha, alpha, alpha, alpha, 10 * alpha])
    # R = np.eye(3) * beta
    R = np.diag([beta, beta, 2.0])

    P_prior = F @ P_prev @ F.T + Q

    S = H @ P_prior @ H.T + R
    K = P_prior @ H.T @ np.linalg.inv(S)
    P = (np.eye(5) - K @ H) @ P_prior

    x = x_prior + K @ (z - H @ x_prior)
    x[3] = normalize_angle(x[3])

    return x, P
