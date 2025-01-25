# from overhead_cv import cv_recorder

# import numpy as np


# def test_calc_pos():
#     tests = [
#         {
#             "input": [
#                 np.array([[5, 5], [20, 5], [20, 20], [5, 20]]),
#                 [500, 500],
#                 500,
#                 500,
#             ],
#             "output": ((12, 12), [(5, 5), (20, 5), (20, 20), (5, 20)]),
#         },
#         {
#             "input": [
#                 np.array([[5, 5], [10, 5], [10, 10], [5, 10]]),
#                 [800, 800],
#                 20,
#                 20,
#             ],
#             "output": ((300, 300), [(200, 200), (400, 200), (400, 400), (200, 400)]),
#         },
#     ]
#     for test in tests:

#         test_input = test["input"]
#         expected_output = test["output"]
#         point, scaled_mesh = cv_recorder.calculate_pos(
#             test_input[0], test_input[1], test_input[2], test_input[3]
#         )
#         assert expected_output[0] == point
#         for point_it in range(4):
#             assert expected_output[1][point_it][0] == scaled_mesh[point_it][0]
#             assert expected_output[1][point_it][1] == scaled_mesh[point_it][1]
