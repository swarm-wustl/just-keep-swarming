import numpy as np
import cv2

from .pixel_to_real import pixel_to_world

# Define the parameters for the transformation
params = {
    "focal_length": 0.01,  # Example focal length
    "cam_height": 1.2573,  # Example camera height
    "fov": (130, 103),  # Field of view in degrees (horizontal, vertical)
    "resolution": (1920, 1080),  # Resolution of the image (width, height)
}

# Create a grid of pixel coordinates
image_width, image_height = params["resolution"]
SPACING = 50  # Spacing between points in pixels
x_vals = np.arange(0, image_width, SPACING)
y_vals = np.arange(0, image_height, SPACING)
x_grid, y_grid = np.meshgrid(x_vals, y_vals)

# Flatten the grid for processing
x_flat = x_grid.flatten()
y_flat = y_grid.flatten()

# Transform each pixel coordinate to world coordinates
world_coords = [pixel_to_world((x, y), params) for x, y in zip(x_flat, y_flat)]
world_x, world_y = zip(*world_coords)

# Create an image to visualize the vector map
image = np.ones((image_height, image_width, 3), dtype=np.uint8) * 255

# Draw vectors showing the transformation
for px, py, wx, wy in zip(x_flat, y_flat, world_x, world_y):
    start_point = (int(px), int(py))  # Pixel coordinates (input)
    end_point = (
        int(px + wx * 100),
        int(py - wy * 100),
    )  # Scaled world coordinates for visualization
    color = (0, 0, 255)  # Red color for the vectors
    THICKNESS = 3  # Thickness of the vector lines
    cv2.arrowedLine(image, start_point, end_point, color, THICKNESS, tipLength=0.3)
    cv2.circle(image, start_point, 5, (255, 0, 0), 5)
    cv2.circle(image, end_point, 5, (0, 255, 0), 5)


# Display the vector map
cv2.imshow("Vector Map", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
