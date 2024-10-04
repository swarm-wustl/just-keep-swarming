import math

import cv2

import numpy as np

import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import Float32MultiArray

CAMERA_ID = 0
DELAY = 1

WINDOW_NAME = "OpenCV QR Code"

MAP_WINDOW = "location tracking"

QCD = cv2.QRCodeDetector()


# intended to convert points to width and height of sub image
def calculate_pos(points, conversion, width, height):

    max_x = max(points[1][0], points[2][0])
    min_x = min(points[0][0], points[3][0])

    max_y = max(points[2][1], points[3][1])
    min_y = min(points[0][1], points[1][1])

    # This will be used to scale (x,y) in pixels to some real unit such as meters.
    # example, if the camera covers 500 by 500 meters, coversion will equal [500, 500]
    point = (
        math.floor(((max_x - (max_x - min_x) / 2) / width) * conversion[0]),
        math.floor(((max_y - (max_y - min_y) / 2) / height) * conversion[1]),
    )

    scaled_mesh_point = points / [width, height] * conversion

    return point, scaled_mesh_point


class CVRecorder(Node):
    def __init__(self):
        super().__init__("cv_recorder")

        self.cv_bridge = CvBridge()

        self.declare_parameter("display", False)
        display_param = self.get_parameter("display").value

        self.display = display_param is True

        self.poses_emit = self.create_publisher(
            Float32MultiArray, "multi_array_pos", 10
        )

        # this is the real life size in your chosen unit,
        # for now we are defaulting to 500 by 500 units
        self.conversion = [500, 500]

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/video",
            callback=self.scan_code_callback,
            qos_profile=10,
        )

        self.robot_points = []
        self.current_robot_points = []

    # changes the point to be relative to the center

    # publishes points and converts ids and points to publish array
    def emit_points(self, ids, points):
        if len(ids) == 0:
            return
        submit = []
        num = len(ids)
        for i in range(num):
            submit.extend([points[i][0], points[i][1], ids[i]])

        submit_data = Float32MultiArray()
        submit_data.data = submit

        self.poses_emit.publish(submit_data)

    def display_images(self, map_image, frame):
        for point in self.robot_points:
            map_image = cv2.circle(
                map_image, point, radius=1, color=(0, 0, 255), thickness=2
            )
        for point in self.current_robot_points:
            map_image = cv2.polylines(
                map_image, [point.astype(int)], True, (0, 255, 0), 3
            )

        cv2.imshow(WINDOW_NAME, frame)

        cv2.imshow(MAP_WINDOW, map_image)
        cv2.waitKey(1)

    def analyze_scan(self, decoded_info, points, dim, frame):
        packet_points = []
        ids = []
        if self.display:
            self.current_robot_points = []
        for code, qr_points in zip(decoded_info, points):

            if not code:
                continue
            color = (0, 255, 0)

            point, scaled_mesh = calculate_pos(
                points=qr_points,
                conversion=self.conversion,
                width=dim[0],
                height=dim[1],
            )
            if self.display:
                frame = cv2.polylines(frame, [qr_points.astype(int)], True, color, 8)

                self.robot_points.append(point)
                self.current_robot_points.append(scaled_mesh)

            relative_point = (
                point[0] - dim[0] / 2,
                point[1] - dim[1] / 2,
            )
            packet_points.append(relative_point)
            ids.append(float(code))
        return ids, packet_points

    # scans QR and gets data from it
    def scan_code_callback(self, data):

        frame = self.cv_bridge.imgmsg_to_cv2(data)

        frame_width = len(frame[0])

        frame_height = len(frame)
        packet_points = []
        ids = []

        ret_qr, decoded_info, points, _ = QCD.detectAndDecodeMulti(frame)
        if ret_qr:

            ids, packet_points = self.analyze_scan(
                decoded_info=decoded_info,
                points=points,
                dim=(frame_width, frame_height),
                frame=frame,
            )

            self.emit_points(ids, packet_points)
        if self.display:
            map_image = np.full(
                (self.conversion[1], self.conversion[0], 3),
                [255, 255, 255],
                dtype=np.uint8,
            )

            self.display_images(frame=frame, map_image=map_image)


def main(args=None):

    print("starting cv_recorder")

    rclpy.init(args=args)

    cv_recorder = CVRecorder()

    rclpy.spin(cv_recorder)

    cv2.destroyWindow(WINDOW_NAME)
