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


# Assuming we are using meters
class CamData:
    def __init__(self, cam_node):

        cam_node.declare_parameter("delay", 0.001)

        self.delay = cam_node.get_parameter("delay").value

        cam_node.declare_parameter("focal_length", -1.0)

        self.focal_length = cam_node.get_parameter("focal_length").value

        cam_node.declare_parameter("cam_height", -1.0)

        self.cam_height = cam_node.get_parameter("cam_height").value

        self.shape = [0.0, 0.0]

        # in future when camera can move
        self.delt_move = (0.0, 0.0)

        self.shape_publisher = cam_node.create_publisher(
            Float32MultiArray, "/img_shape", 10
        )

        self.data_service = cam_node.create_service(
            CamMeta, "cam_meta_data", self.get_meta_data
        )

    def get_meta_data(self, request, response):
        conv = 1.0
        match request.units:
            case "cm":
                conv = 0.01

        response.focal_length = self.focal_length * conv
        response.cam_height = self.cam_height * conv
        response.width = self.shape[0]
        response.height = self.shape[1]
        return response

    def update_size(self, width, height):
        if (self.shape[0], self.shape[1]) == (width, height):
            return

        self.shape[0] = width
        self.shape[1] = height

        update_data = Float32MultiArray()
        update_data.data = self.shape
        print(self.shape)
        self.shape_publisher.publish(update_data)


class CameraFeed(Node):
    def __init__(self):
        super().__init__("camera_feed")
        self.bridge = CvBridge()
        
        self.cam_data = CamData(self)

        if self.cam_data.focal_length <= 0 or self.cam_data.cam_height <= 0:
            raise Exception("valid focal_length and cam_height is a required argument")

        self.cap = cv2.VideoCapture(0)

        self.video_publisher = self.create_publisher(Image, "/video", 10)

        self.timer = self.create_timer(self.cam_data.delay, self.run)

    # gets the camera feed and publishes it to /video topic

    def run(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                return

            # just in case something updates
            self.cam_data.update_size(
                width=float(frame.shape[1]), height=float(frame.shape[0])
            )

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
