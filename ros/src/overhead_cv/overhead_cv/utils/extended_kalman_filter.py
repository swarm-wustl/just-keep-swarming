from typing import Tuple

import numpy as np
from numpy.typing import NDArray


def f(x: NDArray, u: NDArray, dt: float) -> NDArray:
    x_new = np.zeros_like(x)

    cos_theta, sin_theta = x[3], x[4]
    theta_vec = np.array([cos_theta, sin_theta])

    # Update position
    x_new[0] = x[0] + x[2] * cos_theta * dt
    x_new[1] = x[1] + x[2] * sin_theta * dt

    # Update velocity and omega
    x_new[2] = np.clip(x[2] + u[0] * dt, -10.0, 10.0)
    x_new[5] = x[5] + u[1] * dt

    # Update (cos theta, sin theta) using rotation matrix
    dtheta = x[5] * dt
    cos_dtheta = np.cos(dtheta)
    sin_dtheta = np.sin(dtheta)

    R = np.array([[cos_dtheta, -sin_dtheta], [sin_dtheta, cos_dtheta]])

    new_theta_vec = R @ theta_vec
    new_theta_vec /= np.linalg.norm(new_theta_vec)  # normalize

    x_new[3:5] = new_theta_vec
    return x_new


def jacobian(x: NDArray, dt: float) -> NDArray:
    F = np.eye(6)

    v = x[2]
    cos_theta = x[3]
    sin_theta = x[4]
    omega = x[5]

    # ∂x/∂v, ∂x/∂cosθ, ∂x/∂sinθ
    F[0, 2] = cos_theta * dt
    F[0, 3] = v * dt
    F[0, 4] = 0

    F[1, 2] = sin_theta * dt
    F[1, 3] = 0
    F[1, 4] = v * dt

    # ∂[cosθ, sinθ]/∂[cosθ, sinθ, ω] via rotation matrix differential
    dtheta = omega * dt
    cos_d = np.cos(dtheta)
    sin_d = np.sin(dtheta)

    # dR/dθ = R' * ω * dt
    F[3, 3] = cos_d
    F[3, 4] = -sin_d
    F[3, 5] = -sin_theta * dt  # derivative wrt omega

    F[4, 3] = sin_d
    F[4, 4] = cos_d
    F[4, 5] = cos_theta * dt

    # ∂ω/∂ω
    F[5, 5] = 1

    return F


def measurement_model(x: NDArray) -> NDArray:
    # Output x, y, cos(theta), sin(theta)
    return np.array([x[0], x[1], x[3], x[4]])


# pylint:disable=too-many-arguments
def extended_kalman_filter(
    x_prev: NDArray,
    u: NDArray,
    z: NDArray,
    P_prev: NDArray,
    dt: float = 1.0,
    alpha: float = 0.001,
    beta: float = 0.01,
) -> Tuple[NDArray, NDArray]:
    # Convert theta in measurement to [cosθ, sinθ]
    theta = z[2]
    z_vec = np.array([z[0], z[1], np.cos(theta), np.sin(theta)])

    # Predict
    x_prior = f(x_prev, u, dt)
    F = jacobian(x_prev, dt)
    Q = np.diag([alpha, alpha, alpha, alpha, alpha, 30 * alpha])
    P_prior = F @ P_prev @ F.T + Q

    # Measurement update
    H = np.zeros((4, 6))
    H[0, 0] = 1  # ∂z_x / ∂x
    H[1, 1] = 1  # ∂z_y / ∂y
    H[2, 3] = 1  # ∂z_cos / ∂cos
    H[3, 4] = 1  # ∂z_sin / ∂sin

    R = np.diag([beta, beta, 0.01, 0.01])  # low noise on cos/sin
    y = z_vec - measurement_model(x_prior)  # residual

    S = H @ P_prior @ H.T + R
    K = P_prior @ H.T @ np.linalg.inv(S)

    x_post = x_prior + K @ y

    # Normalize orientation vector
    heading = x_post[3:5]
    heading /= np.linalg.norm(heading)
    x_post[3:5] = heading

    P_post = (np.eye(6) - K @ H) @ P_prior

    return x_post, P_post
