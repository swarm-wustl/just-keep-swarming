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
        self.cap = cv2.VideoCapture(0)
        self.publisher = self.create_publisher(Image, "/video", 10)

    # gets the camera feed and publishes it to /video topic
    def run(self):
        while True:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                # cv2.imshow("test", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            except CvBridgeError as error:
                rclpy.shutdown()
                print(error)
                break
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)

    cam_feed = CameraFeed()

    print("publishing camera on /video")

    cam_feed.run()

    cam_feed.destroy_node()
