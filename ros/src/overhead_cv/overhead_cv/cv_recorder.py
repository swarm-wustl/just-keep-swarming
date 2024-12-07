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

WINDOW_NAME = "Camera Feed"

MAP_WINDOW = "location tracking"

QCD = cv2.QRCodeDetector()


def detect_obstacles(frame, lower_color, upper_color):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.imshow("Masked Image", hsv_frame)
    # Create a color mask
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find contours from the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours, mask


def detect_and_draw_boxes(frame, lower_color, upper_color, inverse=False):
    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.imshow("Masked Image", hsv_frame)
    # Create a color mask
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    if inverse:
        mask = cv2.bitwise_not(mask)

    # Find contours from the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, mask


# intended to convert points to width and height of sub image
def calculate_pos(cont_points, conversion, width, height):

    x, y, w, h = cv2.boundingRect(cont_points)
    points = np.array([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])

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
            Float32MultiArray, "robot_array_pos", 10
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
        if len(points) == 0:
            return
        submit = []
        num = len(points)
        for i in range(num):
            submit.extend([ids[i], points[i][0], points[i][1]])

        submit_data = Float32MultiArray()
        submit_data.data = submit

        self.poses_emit.publish(submit_data)

    def emit_obstacles(self, mask_ob):
        if (len(mask_ob) == 0):
            return
        
        mask_ob = mask_ob.flatten()
        mask_ob = mask_ob.astype(float).tolist()

        obstacle_data = Float32MultiArray()
        obstacle_data.data = mask_ob

        self.poses_emit.publish(obstacle_data)
        

    def emit_positions(self, points):
        submit = []
        num = len(points)
        for i in range(num):
            submit.extend([float(points[i][0]), float(points[i][1])])

        # print(submit)
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

    def analyze_scan(self, points, dim, frame, obstacle=False):
        packet_points = []
        if self.display:
            self.current_robot_points = []
        for rob_points in points:
            if cv2.contourArea(rob_points) < 200:
                continue
            color = (0, 255, 0)
            if obstacle:
                color = (255, 0, 0)
            point, scaled_mesh = calculate_pos(
                cont_points=rob_points,
                conversion=self.conversion,
                width=dim[0],
                height=dim[1],
            )
            if self.display:
                frame = cv2.polylines(frame, [rob_points.astype(int)], True, color, 8)

                self.robot_points.append(point)
                self.current_robot_points.append(scaled_mesh)

            relative_point = (point[0], point[1])  # - dim[0] / 2,  # - dim[1] / 2,
            # print(relative_point)
            packet_points.append(relative_point)
        return packet_points

    # scans QR and gets data from it
    def scan_code_callback(self, data):

        frame = self.cv_bridge.imgmsg_to_cv2(data)

        frame_width = len(frame[0])

        frame_height = len(frame)
        packet_points = []

        lower_color = np.array([0, 90, 130])  # Adjust as needed
        upper_color = np.array([18, 255, 255])

        robot_points, mask = detect_and_draw_boxes(frame, lower_color, upper_color)

        inverse_mask = cv2.bitwise_not(mask)

        inverse_frame = cv2.bitwise_and(frame, frame, mask=inverse_mask)

        lower_color_obs = np.array([0, 25, 40])  # Adjust as needed
        upper_color_obs = np.array([255, 255, 255])

        obstacle_points, mask_ob = detect_obstacles(
            inverse_frame, lower_color_obs, upper_color_obs
        )
        if robot_points:

            packet_points = self.analyze_scan(
                points=robot_points,
                dim=(frame_width, frame_height),
                frame=frame,
            )
            self.analyze_scan(
                points=obstacle_points,
                dim=(frame_width, frame_height),
                frame=frame,
                obstacle=True,
            )

            self.emit_positions(packet_points)
            self.emit_obstacles(mask_ob)

        if self.display:
            map_image = np.full(
                (self.conversion[1], self.conversion[0], 3),
                [255, 255, 255],
                dtype=np.uint8,
            )
            result = cv2.bitwise_and(frame, frame, mask=mask)

            result_ob = cv2.bitwise_and(inverse_frame, inverse_frame)

            # cv2.imshow("Mask", mask)
            cv2.imshow("Masked Image", result)

            cv2.imshow("opp Image", result_ob)
            # cv2.imshow("opp Image", result_ob)
            self.display_images(frame=frame, map_image=map_image)


def main(args=None):

    print("starting cv_recorder")

    rclpy.init(args=args)

    cv_recorder = CVRecorder()

    rclpy.spin(cv_recorder)

    cv2.destroyWindow(WINDOW_NAME)
