from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class Measurement:
    x: float
    y: float
    theta: float

    def to_np(self) -> NDArray:
        return np.array([self.x, self.y, self.theta])

    @staticmethod
    def from_np(arr: NDArray):
        return State(*arr)


LEN_STATE_VEC = 6


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    lin_vel: float = 0.0
    cos_theta: float = 0.0
    sin_theta: float = 0.0
    ang_vel: float = 0.0

    def to_np(self) -> NDArray:
        return np.array(
            [self.x, self.y, self.cos_theta, self.sin_theta, self.lin_vel, self.ang_vel]
        )

    @staticmethod
    def from_np(arr: NDArray):
        return State(*arr)


@dataclass
class Command:
    lin_vel: float
    ang_vel: float

    def to_np(self) -> NDArray:
        return np.array([self.lin_vel, self.ang_vel])

    @staticmethod
    def from_np(arr: NDArray):
        return State(*arr)
