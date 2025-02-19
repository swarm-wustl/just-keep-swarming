import cv2
import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from shared_types.srv import CamMeta
from std_msgs.msg import Float32MultiArray


class CameraFeed(Node):
    def __init__(self):
        super().__init__("camera_feed")
        self.bridge = CvBridge()

        self.parameters = {}
        default_values = {
            "delay": 0.001,
            "focal_length": -1.0,
            "cam_height": -1.0,
            "fov_x": -1.0,
            "fov_y": -1.0,
            "resolution_x": -1.0,
            "resolution_y": -1.0,
            "cam_input": 0,
            "shape": [0, 0],
            "dimension": [0.0, 0.0],
        }

        for param, default in default_values.items():
            self.declare_parameter(param, default)
            self.parameters[param] = self.get_parameter(param).value
        self.check_params()

        self.shape_publisher = self.create_publisher(
            Float32MultiArray, "/img_shape", 10
        )

        self.data_service = self.create_service(
            CamMeta, "cam_meta_data", self.get_meta_data
        )

        self.cap = cv2.VideoCapture(f"/dev/video{self.parameters['cam_input']}")

        self.video_publisher = self.create_publisher(Image, "/video", 10)

        self.timer = self.create_timer(self.parameters["delay"], self.publish_video)

        print("Publishing camera on /video")

    def check_params(self):
        if (
            min(
                self.parameters["focal_length"],
                self.parameters["cam_height"],
                self.parameters["fov_x"],
                self.parameters["fov_y"],
                self.parameters["resolution_x"],
                self.parameters["resolution_y"],
            )
            < 0
        ):
            raise ValueError("Not all parameters have been supplied")

    def get_meta_data(self, request, response):
        conv = 0.01 if request.units == "cm" else 1.0

        response.focal_length = self.parameters["focal_length"] * conv
        response.cam_height = self.parameters["cam_height"] * conv
        response.fov_x = self.parameters["fov_x"] * conv
        response.fov_y = self.parameters["fov_y"] * conv
        response.resolution_x = self.parameters["resolution_x"] * conv
        response.resolution_y = self.parameters["resolution_y"] * conv

        response.width = self.parameters["shape"][0]
        response.height = self.parameters["shape"][1]

        return response

    def update_size(self, width, height):
        if (self.parameters["shape"][0], self.parameters["shape"][1]) == (
            width,
            height,
        ):
            return

        self.parameters["shape"][0] = width
        self.parameters["shape"][1] = height

        update_data = Float32MultiArray()
        update_data.data = self.parameters["shape"]
        self.shape_publisher.publish(update_data)

    # Gets the camera feed and publishes it to /video topic
    def publish_video(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                return

            # If something updates
            self.update_size(width=float(frame.shape[1]), height=float(frame.shape[0]))

            self.video_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except CvBridgeError as error:
            rclpy.shutdown()
            print(error)


def main(args=None):
    rclpy.init(args=args)
    cam_feed = CameraFeed()
    rclpy.spin(cam_feed)
    cam_feed.destroy_node()
