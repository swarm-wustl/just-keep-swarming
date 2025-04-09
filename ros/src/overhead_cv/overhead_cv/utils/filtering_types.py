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


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    lin_vel: float = 0.0
    orientation: float = 0.0
    ang_vel: float = 0.0

    def to_np(self) -> NDArray:
        return np.array([self.x, self.y, self.orientation, self.lin_vel, self.ang_vel])

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
