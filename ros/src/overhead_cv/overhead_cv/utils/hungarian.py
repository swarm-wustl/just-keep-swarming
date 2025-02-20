import numpy as np
from scipy.optimize import linear_sum_assignment


def hungarian(current_states, new_measurements):
    """Assigns a measurement to a robot ID"""
    N = len(current_states)
    M = len(new_measurements)
    J = np.zeros((N, M))

    for i in range(N):
        for j in range(M):
            J[i, j] = np.sum((current_states[i] - new_measurements[j]) ** 2)

    _, col_ind = linear_sum_assignment(J)

    return col_ind
