import math

import numpy as np
from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(w, x, y, z):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_from_yaw(yaw_rad) -> Quaternion:
    half_yaw = yaw_rad / 2.0
    cy = np.cos(half_yaw)
    sy = np.sin(half_yaw)

    # Since rotation is only around Z-axis:
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)
