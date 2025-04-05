import numpy as np
import math

# pylint:disable=too-many-locals
def calculate_angle(robot_point, angle_point):
    if(angle_point):
        if((angle_point[0][0] != None) and (angle_point[0][1]!=None)):
            dx = angle_point[0][0] - robot_point[0][0]
            dy = angle_point[0][1] - robot_point[0][1]

            angle_rad = np.arctan2(dy, dx)  # Compute angle in radians

            angle_deg = np.degrees(angle_rad)  # Convert to degrees

            # Normalize to [0, 360)
            angle = (angle_deg + 360) % 360
            return angle
    return float('nan')