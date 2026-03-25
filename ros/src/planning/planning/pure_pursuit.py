from geometry_msgs.msg import Pose, Twist
import numpy as np
import scipy.interpolate as si
from planning.utils import get_yaw_from_pose
import math

def fit_splines(paths: dict[int, np.ndarray]):
    splines = {}
    for robot_id, path in paths.items():
        splines[robot_id] = path # Default to raw path (this is somewhat questionable)
        if path is None or len(path) < 2:
            print(f"bad path given! {path}")
            continue

        # remove points that are too close to the previous one.
        keep_indices = [0]
        for i in range(1, len(path)):
            dist = np.linalg.norm(path[i] - path[keep_indices[-1]])
            if dist > 0.05:  # Minimum 5cm spacing between spline knots
                keep_indices.append(i)
        
        # Always keep the last point/goal
        if keep_indices[-1] != len(path) - 1:
            keep_indices.append(len(path) - 1)
            
        filtered_path = path[keep_indices]

        # Need at least 2 points to make a line
        if len(filtered_path) < 2:
            # splines[robot_id] = path
            continue

        x = filtered_path[:, 0]
        y = filtered_path[:, 1]

        # questionable
        n_points = len(filtered_path)
        if n_points > 3:
            k_val = 3
        elif n_points > 2:
            k_val = 2
        else:
            k_val = 1

        try:
            smoothness = 0.0 if k_val == 1 else 0.01 
            
            tck, u = si.splprep([x, y], k=k_val, s=smoothness)
            u_new = np.linspace(0, 1, 200) # u is normalized 0->1 by default in splprep
            x_smooth, y_smooth = si.splev(u_new, tck)
            splines[robot_id] = np.column_stack((x_smooth, y_smooth))
            
        except Exception as e:
            # If spline fails, fallback to filtered path (better than raw path)
            print(f"Spline fitting failed for robot {robot_id}: {e}")
            splines[robot_id] = filtered_path 

    return splines

def pure_pursuit_control(robot_pose: Pose, path_points: np.ndarray, lookahead_distance: float = 0.6, max_speed: float = 0.8) -> Twist:
    cmd = Twist()
    if path_points is None or len(path_points) < 2:
        return cmd

    rx = robot_pose.position.x
    ry = robot_pose.position.y
    ryaw = get_yaw_from_pose(robot_pose)

    # Find the point on the path closest to the robot
    dists = np.linalg.norm(path_points - np.array([rx, ry]), axis=1)
    closest_idx = np.argmin(dists)

    # Find the Intersection Point (Lookahead)
    target_point = None
    
    for i in range(closest_idx, len(path_points)):
        dist = dists[i]
        if dist >= lookahead_distance:
            target_point = path_points[i]
            break
            
    # If we are near the end and can't find a point far enough away, take the last one
    if target_point is None:
        target_point = path_points[-1]

    # Transform goal to vehicle frame
    dx = target_point[0] - rx
    dy = target_point[1] - ry
    
    local_x = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
    local_y = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy

    if local_x < 0:
        # Option A: If we are really close to the final goal, just turn to face it
        if np.linalg.norm([dx, dy]) < 1.0:
             v = 0.0
             w = 0.5 if local_y > 0 else -0.5 # Spin in place
             cmd.linear.x = v
             cmd.angular.z = w
             return cmd
        else:
             # Option B: We are lost/off-path. Just drive forward slowly to re-acquire 
             # or treating it as a sharp turn. Let's force a sharp turn.
             local_x = 0.01 # Hack to avoid division by zero and force high curvature

    # 4. Calculate Curvature (gamma = 2y / L^2)
    L2 = local_x**2 + local_y**2
    
    # Avoid division by zero
    if L2 < 0.001:
        return cmd

    gamma = 2 * local_y / L2

# 5. Calculate "Raw" Target Velocities
    # Calculate the desired raw linear velocity
    dist_to_goal = np.linalg.norm(path_points[-1] - np.array([rx, ry]))
    
    raw_v = max_speed
    if dist_to_goal < 0.5:
        raw_v = max_speed * (dist_to_goal / 0.5)
        # raw_v = max(0.1, raw_v) # help reduce steady state erro
        # TODO: PI control (integral will help)
        # if raw_v < 0.1: raw_v = 0.0

    # Calculate the desired raw angular velocity based on curvature
    # w = v * gamma
    raw_w = raw_v * gamma

    # 6. Normalize to limits (Preserving Curvature)
    MAX_LIN_VEL = 0.5  # Your limit
    MAX_ANG_VEL = 1.0  # Your limit

    # Initialize scale to 1.0 (no scaling)
    scale = 1.0

    # Check if linear velocity exceeds limit
    if abs(raw_v) > MAX_LIN_VEL:
        scale = min(scale, MAX_LIN_VEL / abs(raw_v))

    # Check if angular velocity exceeds limit (CRITICAL STEP)
    # If the turn is too sharp for our max motor speed, we MUST slow down v
    if abs(raw_w) > MAX_ANG_VEL:
        scale = min(scale, MAX_ANG_VEL / abs(raw_w))

    # Apply the same scale to both to preserve the arc
    v = raw_v * scale
    w = raw_w * scale

    cmd.linear.x = v
    cmd.angular.z = w

    # print(v, w)
    return cmd
