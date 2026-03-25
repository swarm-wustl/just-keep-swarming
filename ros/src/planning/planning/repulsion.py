import math

from geometry_msgs.msg import Pose, Twist

from planning.utils import get_yaw_from_pose

def apply_repulsion(
    original_cmd: Twist, 
    current_pose: Pose, 
    all_poses: dict, 
    my_robot_id: int, 
    repulsion_radius: float = 1.2,
    repulsion_gain: float = 0.8,
    swirl_gain: float = 1.2,
    lateral_shrink: float = 0.5
) -> Twist:
    rx = current_pose.position.x
    ry = current_pose.position.y
    ryaw = get_yaw_from_pose(current_pose)
    c = math.cos(ryaw)
    s = math.sin(ryaw)

    fx_global = 0.0
    fy_global = 0.0
    
    nearby_robot_count = 0
    max_danger_level = 0.0
    
    # 0.0 = No Limit, 1.0 = No Left, -1.0 = No Right
    turn_constraint = 0.0 
    
    is_hard_blocked = False
    closest_obs_angle = 0.0

    for other_id, other_pose in all_poses.items():
        if other_id == my_robot_id: continue

        ox = other_pose.position.x
        oy = other_pose.position.y
        
        dx_global = rx - ox
        dy_global = ry - oy
        
        dist_sq = dx_global**2 + dy_global**2
        if dist_sq > repulsion_radius**2: continue

        # Local Transform
        dx_local = c * dx_global + s * dy_global
        dy_local = -s * dx_global + c * dy_global

        # Ignore Rear (Vision Cone)
        if dx_local < -0.3: continue

        safe_shrink = max(0.1, lateral_shrink)
        dist_elliptical = math.sqrt((dx_local)**2 + (dy_local / safe_shrink)**2)

        if dist_elliptical < repulsion_radius:
            nearby_robot_count += 1
            
            angle_to_obs = math.atan2(dy_local, dx_local)
            dist_real = math.sqrt(dx_global**2 + dy_global**2)

            # --- 1. RED ZONE (The Wall) ---
            if abs(angle_to_obs) < 0.52 and dist_real < (repulsion_radius * 0.9):
                is_hard_blocked = True
                closest_obs_angle = angle_to_obs

            # --- 2. YELLOW ZONE (The Shadow) ---
            elif abs(angle_to_obs) < 1.6:
                if angle_to_obs > 0:
                    if turn_constraint == 0: turn_constraint = -1 # Block Left (Pos)
                else:
                    if turn_constraint == 0: turn_constraint = 1  # Block Right (Neg)

            # --- 3. REPULSION FORCES ---
            # gain_scale = 1.5 if my_robot_id < other_id else 0.4 # bully logic
            gain_scale = 1.0
            
            danger = 1.0 - (dist_elliptical / repulsion_radius)
            max_danger_level = max(max_danger_level, danger)

            inv_dist = 1.0 / max(0.01, dist_elliptical)
            
            # Quadratic repulsion for stronger reaction at close range
            mag_rep = repulsion_gain * gain_scale * (inv_dist - (1.0/repulsion_radius)) * (inv_dist**2)

            vec_away_x = dx_global / max(0.01, dist_real)
            vec_away_y = dy_global / max(0.01, dist_real)

            fx_global += vec_away_x * mag_rep
            fy_global += vec_away_y * mag_rep
            
            # Add Swirl (Tangential Force) to aid orbiting
            # If obstacles < 2, we can afford to swirl more aggressively
            if nearby_robot_count < 2:
                fx_global += vec_away_y * mag_rep * swirl_gain
                fy_global += -vec_away_x * mag_rep * swirl_gain

    if nearby_robot_count == 0:
        return original_cmd

    modified_cmd = Twist()

    # --- CASE A: HARD BLOCK (Red Zone) ---
    if is_hard_blocked:
        modified_cmd.linear.x = 0.0
        turn_dir = -1.0 if closest_obs_angle >= 0 else 1.0
        modified_cmd.angular.z = turn_dir * 0.8
        return modified_cmd

    # --- CASE B: SOFT BLOCK (Yellow/Green Zone) ---
    else:
        # Transform Repulsion forces to Local Frame
        force_x_local = c * fx_global + s * fy_global
        force_y_local = -s * fx_global + c * fy_global

        alpha = max(0.0, 1.0 - (max_danger_level * 1.5))
        
        # --- FIX: STUCK DETECTION (Equilibrium Breaker) ---
        # Calculate what the net forward velocity WOULD be
        proposed_forward_effort = (original_cmd.linear.x * alpha) + force_x_local

        # If we are in high danger and the repulsion is cancelling our drive...
        # We switch to "Orbit Mode": Kill linear, maximize rotation using lateral force.
        if proposed_forward_effort < 0.05 and max_danger_level > 0.5:
            
            modified_cmd.linear.x = 0.0 # Don't grind against the field
            
            # Use the lateral force (swirl) to dictate rotation
            # Amplify it significantly to force the robot around the obstacle
            orbit_turn = force_y_local * 4.0 
            
            # Clamp it so we don't spin like a top, but ensure it's at least moving
            if abs(orbit_turn) < 0.3:
                # If lateral force is too weak, force a default turn based on constraint
                if turn_constraint == -1: orbit_turn = -0.5
                elif turn_constraint == 1: orbit_turn = 0.5
                else: orbit_turn = 0.5 # Default Left
            
            base_w = orbit_turn

        else:
            # Standard blended behavior
            base_v = proposed_forward_effort
            base_w = (original_cmd.angular.z * alpha) + (force_y_local * 3.0)
            modified_cmd.linear.x = max(-0.5, min(base_v, 0.5))

        # --- APPLY CONSTRAINTS ---
        if turn_constraint == -1: # Block Left
            base_w = min(0.0, base_w)
        elif turn_constraint == 1: # Block Right
            base_w = max(0.0, base_w)

        modified_cmd.angular.z = max(-1.5, min(base_w, 1.5))

        return modified_cmd
