
import numpy as np

def pixel_to_world(pixel_coords, focal_length, camera_height, sensor_size, image_resolution):
    x, y = pixel_coords
    sensor_width, sensor_height = sensor_size
    image_width, image_height = image_resolution
   
    x_norm = x * (sensor_width / image_width)
    y_norm = y (sensor_height / image_height)
   
    X = (x_norm * camera_height) / focal_length
    Y = (y_norm * camera_height) / focal_length
   
    return X, Y