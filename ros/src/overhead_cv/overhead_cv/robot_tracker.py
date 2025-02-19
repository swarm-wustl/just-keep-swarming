import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from shared_types.msg import RobotPoints

from .utils.img_proc_utils import calculate_pos, detect_and_draw_boxes

CAMERA_ID = 0
DELAY = 1

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
            RobotPoints, "robot_observations", 10
        )

        self.create_subscription(
            msg_type=Image,
            topic="/video",
            callback=self.scan_code_callback,
            qos_profile=10,
        )

        self.tracked_points = []  # Historical positions
        self.current_contours_points = []  # Current frame's contour data
        print("Tracking robots")

    def emit_positions(self, points):
        """
        Publish the detected robot positions
        """
        msg = RobotPoints()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.points = [Point(x=float(pt[0]), y=float(pt[1]), z=0.0) for pt in points]

        self.positions_publisher.publish(msg)

    def display_images(self, map_image, frame):
        """
        Overlay detected points and contours onto the map and display both
        the original frame and the generated map image
        """
        # Historical tracked points
        for point in self.tracked_points:
            cv2.circle(map_image, point, radius=1, color=(0, 0, 255), thickness=2)

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

    def analyze_contours(self, contours, frame_dims, frame):
        """
        Process detected contours to compute robot positions.
        """
        packet_points = []
        if self.display:
            # Reset the list for the current frame's contours
            self.current_contours_points = []

        for contour in contours:
            if cv2.contourArea(contour) < 200:
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

            packet_points.append((point[0], point[1]))

        return packet_points

    def scan_code_callback(self, data):
        """
        Callback for incoming image data. Converts the image, detects contours,
        computes positions, and publishes the results
        """
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Get frame dimensions (width, height)
        frame_height, frame_width = frame.shape[:2]

        # Define HSV color bounds for detection (adjust as necessary)
        lower_color = np.array([0, 124, 0])
        upper_color = np.array([18, 255, 255])

        # Detect contours corresponding to the robot markers and obtain the mask
        detected_contours, mask = detect_and_draw_boxes(frame, lower_color, upper_color)

        if detected_contours:
            packet_points = self.analyze_contours(
                contours=detected_contours,
                frame_dims=(frame_width, frame_height),
                frame=frame,
            )
            self.emit_positions(packet_points)

        if self.display:
            # Create a blank white map image for tracking display
            map_image = np.full(
                (self.conversion[1], self.conversion[0], 3), 255, dtype=np.uint8
            )
            result = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow("Mask", mask)
            cv2.imshow("Masked Image", result)
            self.display_images(map_image=map_image, frame=frame)


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
