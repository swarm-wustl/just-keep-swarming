from typing import List

import numpy as np
from scipy.optimize import linear_sum_assignment

from overhead_cv.utils.filtering_types import Measurement, State


def hungarian(
    current_states: List[State], new_measurements: List[Measurement]
) -> List[int]:
    """
    Assigns a measurement to a robot ID.
    current_states is shape Nx2 and new_measurements is shape Mx2
    """
    N = len(current_states)
    M = len(new_measurements)
    J = np.zeros((N, M))

    for i in range(N):
        for j in range(M):
            # Only get (x, y) values from the state and measurement
            x = current_states[i].to_np()[:2]
            z = new_measurements[j].to_np()[:2]
            J[i, j] = np.sum((x - z) ** 2)

    _, col_ind = linear_sum_assignment(J)

    return col_ind
