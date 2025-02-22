import math
from typing import Sequence, Tuple

import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray


def detect_and_draw_boxes(
    frame: MatLike, lower_color: NDArray, upper_color: NDArray
) -> Tuple[Sequence[MatLike], MatLike]:
    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.imshow("Masked Image", hsv_frame)

    # Create a color mask
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find contours from the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours, mask


# intended to convert points to width and height of sub image
def calculate_pos(
    cont_points, conversion, width, height
) -> Tuple[Tuple[int, int], MatLike]:
    x, y, w, h = cv2.boundingRect(cont_points)
    points = np.array([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])

    max_x = max(points[1][0], points[2][0])
    min_x = min(points[0][0], points[3][0])

    max_y = max(points[2][1], points[3][1])
    min_y = min(points[0][1], points[1][1])

    # This will be used to scale (x,y) in pixels to some real unit such as meters.
    # example, if the camera covers 500 by 500 meters, coversion will equal [500, 500]
    point = (
        math.floor(((max_x - (max_x - min_x) / 2) / width) * conversion[0]),
        math.floor(((max_y - (max_y - min_y) / 2) / height) * conversion[1]),
    )

    scaled_mesh_point = points / [width, height] * conversion

    return point, scaled_mesh_point
