import numpy as np


# https://arxiv.org/pdf/1708.05551
def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi
