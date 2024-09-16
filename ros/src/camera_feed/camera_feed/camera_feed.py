import cv2

import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CameraFeed(Node):
    def __init__(self):
        super().__init__("camera_feed")
        self.bridge = CvBridge()

        self.declare_parameter("delay", 0.001)
        delay = self.get_parameter("delay").value
        self.cap = cv2.VideoCapture(0)
        self.publisher = self.create_publisher(Image, "/video", 10)
        self.timer = self.create_timer(delay, self.run)

    # gets the camera feed and publishes it to /video topic
    def run(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                return
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

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
