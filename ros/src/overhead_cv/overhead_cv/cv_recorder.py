import math
from cv2 import cv2

import numpy as np

import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

CAMERA_ID = 0
DELAY = 1

WINDOW_NAME = "OpenCV QR Code"

MAP_WINDOW = "location tracking"

QCD = cv2.QRCodeDetector()
CAP = cv2.VideoCapture(CAMERA_ID)


class CVRecorder(Node):
    def __init__(self):
        super().__init__("cv_recorder")
        self.poses_emit = self.create_publisher(
            Float32MultiArray, "multi_array_pos", 10
        )

        self.robot_points = []

    # intended to convert points to width and height of sub image
    # pylint: disable=R0201
    def draw_robot(self, points, conversion, width, height):

        max_x = max(points[1][0], points[2][0])
        min_x = min(points[0][0], points[3][0])

        max_y = max(points[2][1], points[3][1])
        min_y = min(points[0][1], points[1][1])

        point = (
            math.floor(((max_x - (max_x - min_x) / 2) / width) * conversion[0]),
            math.floor(((max_y - (max_y - min_y) / 2) / height) * conversion[1]),
        )

        scaled_mesh_point = np.array(
            [
                ((points[0] / [width, height]) * conversion),
                (points[1] / [width, height]) * conversion,
                (points[2] / [width, height]) * conversion,
                (points[3] / [width, height]) * conversion,
            ]
        )

        return [point, scaled_mesh_point]

    # changes the point to be relative to the center

    # publishes points and converts ids and points to publish array
    def emit_points(self, ids, points):
        if len(ids) == 0:
            return
        submit = []
        num = len(ids)
        for i in range(num):
            submit = submit + [points[i][0], points[i][1]] + [ids[i]]

        submit_data = Float32MultiArray()
        submit_data.data = submit
        # print(submit)

        self.poses_emit.publish((submit_data))

    # scans QR and gets data from it
    # pylint: disable=R0914
    def scan_qr(self):
        ret, frame = CAP.read()

        status_width = len(frame[0])

        status_height = len(frame)
        packet_points = []
        ids = []

        conversion = [status_width, status_height]
        map_image = np.full(
            (status_height, status_width, 3), [255, 255, 255], dtype=np.uint8
        )

        if ret:

            ret_qr, decoded_info, points, _ = QCD.detectAndDecodeMulti(frame)
            if ret_qr:

                for code, point in zip(decoded_info, points):

                    if code:
                        color = (0, 255, 0)
                    else:
                        continue

                    frame = cv2.polylines(frame, [point.astype(int)], True, color, 8)
                    robo_data = self.draw_robot(
                        points=point,
                        conversion=conversion,
                        width=conversion[0],
                        height=conversion[1],
                    )

                    self.robot_points.append(robo_data[0])

                    relative_point = (
                        robo_data[0][0] - conversion[0] / 2,
                        robo_data[0][1] - conversion[1] / 2,
                    )
                    packet_points = packet_points + [relative_point]
                    ids = ids + [float(code)]

                    map_image = cv2.polylines(
                        map_image, [robo_data[1].astype(int)], True, color, 3
                    )

            for point in self.robot_points:
                map_image = cv2.circle(
                    map_image, point, radius=1, color=(0, 0, 255), thickness=2
                )

            self.emit_points(ids, packet_points)

            cv2.imshow(WINDOW_NAME, frame)

            cv2.imshow(MAP_WINDOW, map_image)


def main():

    print("starting cv_recorder")
    rclpy.init()
    cv_recorder = CVRecorder()

    while True:
        cv_recorder.scan_qr()
        if cv2.waitKey(DELAY) & 0xFF == ord("q"):
            break

    cv2.destroyWindow(WINDOW_NAME)
