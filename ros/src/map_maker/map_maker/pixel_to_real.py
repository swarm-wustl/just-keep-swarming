import numpy as np

def pixel_to_world(pixel_coords, focal_length, camera_height, fov, resolution):
    x, y = pixel_coords

    fov_x, fov_y = fov
    image_width, image_height = resolution

    fov_h = np.deg2rad(fov_x)
    fov_v = np.deg2rad(fov_y)

    width_real = 2 * camera_height * np.tan(fov_h / 2)
    height_real = 2 * camera_height * np.tan(fov_v / 2)

    x_shifted = x - image_width / 2
    y_shifted = y - image_height / 2

    x_norm = (x_shifted / (image_width / 2)) * (width_real / 2)
    y_norm = (y_shifted / (image_height / 2)) * (height_real / 2)

    X = (x_norm * camera_height) / focal_length
    Y = (y_norm * camera_height) / focal_length

    return X, Y
