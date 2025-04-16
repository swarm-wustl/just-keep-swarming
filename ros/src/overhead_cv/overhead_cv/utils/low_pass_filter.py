def low_pass_filter(new_angle, prev_filtered, alpha=0.15):
    return alpha * new_angle + (1 - alpha) * prev_filtered
