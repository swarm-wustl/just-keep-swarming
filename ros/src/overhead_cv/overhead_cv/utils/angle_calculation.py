import numpy as np


# pylint:disable=too-many-locals
def calculate_angle(robot_point, angle_point):
    if angle_point:
        if angle_point[0][0] is not None and angle_point[0][1] is not None:
            dx = angle_point[0][0] - robot_point[0][0]
            dy = angle_point[0][1] - robot_point[0][1]

            angle = np.arctan2(dy, dx)  # Compute angle in radians

            # angle_deg = np.degrees(angle_rad)  # Convert to degrees

            # Normalize to [0, 2pi)
            offset = 25 * np.pi / 180  # because the blue is in the corner
            angle = (2 * np.pi - angle) % (2 * np.pi) - offset
            return angle

    return 0  # float("nan") # TODO(sebtheiler): this is bad, nan is better, but works for now
