import numpy as np


def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi
