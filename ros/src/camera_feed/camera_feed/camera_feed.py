import cv2

import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from shared_types.srv import CamMeta

from std_msgs.msg import Float32MultiArray


class CameraFeed(Node):
    def __init__(self):
        super().__init__("camera_feed")
        self.bridge = CvBridge()

        self.parameters = {
            "delay": -1,
            "focal_length": -1.0,
            "cam_height": -1.0,
            "fov_x": -1.0,
            "fov_y": -1.0,
            "resolution_x": -1.0,
            "resolution_y": -1.0,
            "shape": [0, 0],
        }
        # region pamaters

        self.declare_parameter("delay", 0.001)

        self.parameters["delay"] = self.get_parameter("delay").value

        self.declare_parameter("focal_length", -1.0)

        self.parameters["focal_length"] = self.get_parameter("focal_length").value

        self.declare_parameter("cam_height", -1.0)

        self.parameters["cam_height"] = self.get_parameter("cam_height").value

        self.declare_parameter("fov_x", -1.0)

        self.parameters["fov_x"] = self.get_parameter("fov_x").value

        self.declare_parameter("fov_y", -1.0)

        self.parameters["fov_y"] = self.get_parameter("fov_y").value

        self.declare_parameter("resolution_x", -1.0)

        self.parameters["resolution_x"] = self.get_parameter("resolution_x").value

        self.declare_parameter("resolution_y", -1.0)

        self.parameters["resolution_y"] = self.get_parameter("resolution_y").value

        self.declare_parameter("cam_input", 0)

        cam_input_id = self.get_parameter("cam_input").value

        self.parameters["dimension"] = [0.0, 0.0]

        # in future when camera can move
        # self.parameters.delt_move = (0.0, 0.0)
        # endregion

        self.shape_publisher = self.create_publisher(
            Float32MultiArray, "/img_shape", 10
        )

        self.data_service = self.create_service(
            CamMeta, "cam_meta_data", self.get_meta_data
        )

        self.check_params()

        self.cap = cv2.VideoCapture(f"/dev/video{cam_input_id}")

        self.video_publisher = self.create_publisher(Image, "/video", 10)

        self.timer = self.create_timer(self.parameters["delay"], self.run)

    # gets the camera feed and publishes it to /video topic
    # pylint:disable=too-many-boolean-expressions
    def check_params(self):
        if (
            self.parameters["delay"] <= 0
            or self.parameters["focal_length"] <= 0
            or self.parameters["cam_height"] <= 0
            or self.parameters["fov_x"] <= 0
            or self.parameters["fov_y"] <= 0
            or self.parameters["resolution_x"] <= 0
            or self.parameters["resolution_y"] <= 0
        ):
            raise Exception("incorrect parameters")

    def get_meta_data(self, request, response):
        conv = 1.0
        match request.units:
            case "cm":
                conv = 0.01

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
        print(update_data.data)
        self.shape_publisher.publish(update_data)

    def run(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                return

            # just in case something updates
            self.update_size(width=float(frame.shape[1]), height=float(frame.shape[0]))

            self.video_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except CvBridgeError as error:
            rclpy.shutdown()
            print(error)


def main(args=None):
    rclpy.init(args=args)

    cam_feed = CameraFeed()

    print("publishing camera on /video")

    cam_feed.run()
    rclpy.spin(cam_feed)

    cam_feed.destroy_node()
