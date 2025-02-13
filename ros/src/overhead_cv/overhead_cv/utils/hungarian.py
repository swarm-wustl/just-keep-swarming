import numpy as np
from scipy.optimize import linear_sum_assignment

# def hungarian(xs, us, zs, estimators, dt=1):
#     N = xs.shape[0]
#     Nz = zs.shape[0]
#     if N > Nz:
#         zs += [None] * (N - Nz)
#
#     J = np.zeros((N, N))
#     for i in range(N):
#         for j in range(N):
#             if zs[j] is None:
#                 J[i, j] = np.Inf
#             else:
#                 x_prev = xs[i]
#                 x_cur = estimators[j].update_estimate(us[j], zs[j], dt, save=False)
#                 J[i, j] = np.sum((x_prev - x_cur) ** 2)
#
#     _, col_ind = linear_sum_assignment(J)
#
#     reordered_ids = [estimators[i].id for i in col_ind]
#
#     return reordered_ids


# TODO(sebtheiler): this whole system needs a rework # pylint: disable=fixme
def hungarian(xs_prev, xs_cur, ids):
    N = len(xs_prev)  # xs_prev.shape[0]
    assert N == len(xs_cur)
    J = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            if xs_cur[j] is None:
                J[i, j] = 999999  # np.Inf
            else:
                J[i, j] = np.sum((xs_prev[i] - xs_cur[j]) ** 2)

    _, col_ind = linear_sum_assignment(J)

    reordered_ids = [ids[i] for i in col_ind]

    return reordered_ids
