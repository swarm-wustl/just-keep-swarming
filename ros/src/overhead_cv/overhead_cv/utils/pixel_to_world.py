import os
from typing import Tuple

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

param_file = os.path.join(
    get_package_share_directory("overhead_cv"),
    "config",
    "cam_meta.yaml",
)
with open(param_file) as stream:
    default_params = yaml.safe_load(stream)


# focal_length, camera_height, fov, resolution
# pylint:disable=too-many-locals
def pixel_to_world(
    pixel_coords: Tuple[int, int], *, params=default_params
) -> Tuple[float, float]:
    camera_height = params["cam_height"]
    fov_x = params["fov_x"]
    fov_y = params["fov_y"]
    image_width = params["resolution_x"]
    image_height = params["resolution_y"]

    x, y = pixel_coords

    fov_h = np.deg2rad(fov_x)
    fov_v = np.deg2rad(fov_y)

    width_real = 2 * camera_height * np.tan(fov_h / 2)
    height_real = 2 * camera_height * np.tan(fov_v / 2)

    x_shifted = x - image_width / 2
    y_shifted = y - image_height / 2

    x_norm = (x_shifted / (image_width / 2)) * (width_real / 2)
    y_norm = (y_shifted / (image_height / 2)) * (height_real / 2)

    X = x_norm * camera_height  # / focal_length
    Y = y_norm * camera_height  # / focal_length

    return -(X + 2.5), (Y + 1.0)
