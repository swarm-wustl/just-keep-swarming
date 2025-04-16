from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from overhead_cv.utils.pixel_to_world import pixel_to_world
from overhead_cv.utils.angle_calculation import calculate_angle
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
import math

from .utils.img_proc_utils import calculate_pos, detect_and_draw_boxes

WINDOW_NAME = "Camera Feed"
MAP_WINDOW = "Location Tracking"


class RobotTracker(Node):
    def __init__(self):
        super().__init__("robot_tracker")
        self.cv_bridge = CvBridge()

        # Conversion factors for mapping (defaulting to 500 x 500 units)
        self.conversion = [500, 500]

        self.declare_parameter("display", False)
        self.display = self.get_parameter("display").value is True

        self.positions_publisher = self.create_publisher(
            PoseArray, "robot_observations", 10
        )

        self.create_subscription(
            msg_type=Image,
            topic="/video",
            callback=self.image_callback,
            qos_profile=10,
        )

        self.draw_pairs = []
        self.tracked_points = []  # Historical positions
        self.current_contours_points = []  # Current frame's contour data
        self.get_logger().info("Tracking robots")

    def create_pose(self, x, y, theta):
        q = Quaternion()

        if(theta!='nan'):
            theta=math.radians(theta)
            q.w = math.cos(theta / 2)
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(theta / 2)
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)  # Z is usually 0 for 2D poses
        pose.orientation = q
        
        return pose

    def publish_pixels_as_positions(self, points):
        """
        Publish the detected robot positions as real world positions
        """

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        points = [
            (xx, yy, 0.0 if z == 'nan' else z) 
            for (xx, yy), (_, _, z) in zip(map(pixel_to_world, [(x, y) for x, y, _ in points]), points)
        ]
        msg.poses = [self.create_pose(*point) for point in points]
        self.positions_publisher.publish(msg)

    def display_images(self, map_image, frame):
        """
        Overlay detected points and contours onto the map and display both
        the original frame and the generated map image
        """

        # Historical tracked points
        for point in self.tracked_points:
            cv2.circle(map_image, point, radius=1, color=(0, 0, 255), thickness=2)

        # Colored center and angle points
        count = 0
        for (center_xy, angle_xy) in enumerate(self.draw_pairs):
            if count % 3 == 0:
                cv2.circle(map_image, (int(center_xy[0]), int(center_xy[1])), radius = 1, color=(255, 0, 0), thickness = 1)
                cv2.circle(map_image, (int(angle_xy[0]), int(angle_xy[1])), radius = 1, color=(255, 0, 0), thickness = 1) 
            elif count % 3 == 1:
                cv2.circle(map_image, (int(center_xy[0]), int(center_xy[1])), radius = 1, color=(0, 255, 0), thickness = 1)
                cv2.circle(map_image, (int(angle_xy[0]), int(angle_xy[1])), radius = 1, color=(0, 255, 0), thickness = 1) 
            else:
                cv2.circle(map_image, (int(center_xy[0]), int(center_xy[1])), radius = 1, color=(0, 0, 255), thickness = 1)
                cv2.circle(map_image, (int(angle_xy[0]), int(angle_xy[1])), radius = 1, color=(0, 0, 255), thickness = 1)

            count += 1 

        # Current frame contours
        for contour in self.current_contours_points:
            cv2.polylines(
                map_image,
                [contour.astype(int)],
                isClosed=True,
                color=(0, 255, 0),
                thickness=3,
            )

        cv2.imshow(WINDOW_NAME, frame)
        cv2.imshow(MAP_WINDOW, map_image)
        cv2.waitKey(1)

    def analyze_contours(self, contours, frame_dims, contour_size, frame) -> List[Tuple[int, int]]:
        """
        Process detected contours to compute robot positions.
        """
        packet_points = []
        if self.display:
            # Reset the list for the current frame's contours
            self.current_contours_points = []

        for contour in contours:
            # Check if contour is the expected size
            if cv2.contourArea(contour) < contour_size:
                continue
            # Calculate the position and a scaled version of the contour for display
            point, scaled_mesh = calculate_pos(
                cont_points=contour,
                conversion=self.conversion,
                width=frame_dims[0],
                height=frame_dims[1],
            )

            if self.display:
                # Overlay the contour on the original frame for visualization
                cv2.polylines(
                    frame,
                    [contour.astype(int)],
                    isClosed=True,
                    color=(0, 255, 0),
                    thickness=8,
                )
                self.tracked_points.append(point)
                self.current_contours_points.append(scaled_mesh)
            packet_points.append(point)

        return packet_points

    def image_callback(self, data):
        """
        Callback for incoming image data. Converts the image, detects contours,
        computes positions, and publishes the results
        """
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        packet_points = []

        # Get frame dimensions (width, height)
        frame_height, frame_width = frame.shape[:2]

        # Define HSV orange color bounds for detection (adjust as necessary) 
        lower_color = np.array([0, 0.20*255, 0.55*255])
        upper_color = np.array([30, 0.70*255, 1*255])

        # Detect contours corresponding to the robot markers and obtain the mask
        detected_contours, mask = detect_and_draw_boxes(frame, lower_color, upper_color)


        map_image = np.full(
            (self.conversion[1], self.conversion[0], 3), 255, dtype=np.uint8
        )
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Mask", mask)
        cv2.imshow("Masked Image", result)
        self.display_images(map_image=map_image, frame=frame)


        if detected_contours:
            for i, robot_contour in enumerate(detected_contours):
                packet_point = self.analyze_contours(
                    contours=[robot_contour],
                    frame_dims=(frame_width, frame_height),
                    contour_size=200,
                    frame=frame,
                )
                if packet_point:

                    #if cv2.contourArea(robot_contour) >= 200:
                    #differenting between multiple robots in frame
                #  for packet_point in packet_points:
                    x, y, w, h = cv2.boundingRect(robot_contour)

                    x_start = int(x - (w / 2))
                    x_end = int(x + w * 1.5)
                    y_start = int(y - (h / 2))
                    y_end = int(y + h * 1.5)

                    # Create a frame that masks around each robot contour to only find the related blue point
                    frame_angle = np.ones_like(frame, dtype=np.uint8) * 255
                    frame_angle[y_start:y_end, x_start:x_end] = frame[y_start:y_end, x_start:x_end]
                    
                    # Define HSV color bounds for detection (adjust as necessary)
                    angle_lower_color = np.array([180/2,0.2*255,0.45*255])
                    angle_upper_color = np.array([230/2,0.90*255,0.90*255])

                    angle_points, angle_mask = detect_and_draw_boxes(frame_angle, angle_lower_color, angle_upper_color)
                    
                    if angle_points:
                        angle_packet_point = self.analyze_contours(
                            contours=angle_points,
                            frame_dims=(frame_width, frame_height),
                            contour_size=0, # adjust
                            frame=frame,
                        )
                    else:
                        angle_packet_point= [[None, None]]
                    angle = calculate_angle(packet_point, angle_packet_point)

                    packet_points.append([packet_point[0][0],packet_point[0][1],angle])

                    center_xy = packet_point[0]
                    angle_xy = angle_packet_point[0]
                    self.draw_pairs.append((center_xy, angle_xy))


                    if self.display:
                        # Create a blank white map image for tracking display
                        map_image = np.full(
                            (self.conversion[1], self.conversion[0], 3), 255, dtype=np.uint8
                        )
                        result = cv2.bitwise_and(frame, frame, mask=mask)

                        cv2.imshow("Mask", mask)
                        cv2.imshow("Angle Mask", angle_mask)
                        cv2.imshow("Masked Image", result)
                        
                        self.display_images(map_image=map_image, frame=frame)

            self.publish_pixels_as_positions(packet_points)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
