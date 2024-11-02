from map_maker import pixel_to_real
import numpy as np


def test_projection():
    tests = [
        {
            "input": [
                (5,5),#np.array([[5, 5], [20, 5], [20, 20], [5, 20]]),
                35.0,  # Focal length in mm
                1.5,  # Camera height in meters
                #(36.0, 24.0),  # Sensor size in mm (width, height)
                #(1920, 1080)  # Image resolution in pixels (width, height)
            ],
            "output": (# np.array([  # Expected world coordinates (in meters)
                (0.01071429, 0.00642857)#[0.01071429, 0.00642857],
                #[0.04285714, 0.00642857],
                #[0.04285714, 0.02571429],
                #[0.01071429, 0.02571429]
            )
        },
        {
            "input": [
                (100,5),#np.array([[100, 50], [200, 50], [200, 150], [100, 150]]),
                50.0,  # Focal length in mm
                2.0,  # Camera height in meters
               # (36.0, 24.0),  # Sensor size in mm (width, height)
                #(1920, 1080)  # Image resolution in pixels (width, height)
            ],
            "output": (#np.array([  # Expected world coordinates (in meters)
                (0.1875, 0.05555556)#[0.1875, 0.05555556],
                #[0.375, 0.05555556],
                #[0.375, 0.16666667],
                #[0.1875, 0.16666667]
            )
        }
    ]
    
    for test in tests:
        test_input = test["input"]
        expected_output = test["output"]
        
        # Calling the pixel_to_world function
        result = pixel_to_real.pixel_to_world(
            test_input[0],  # Pixel coordinates
            test_input[1],  # Focal length
            test_input[2],  # Camera height
            #test_input[3],  # Sensor size
            #test_input[3]   # Image resolution
        )
        print(result)
        print("hello")
        # Compare the output to the expected result
        np.testing.assert_almost_equal(result, expected_output, decimal=2)
