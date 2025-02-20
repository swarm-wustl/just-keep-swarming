import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from shared_types.msg import RobotPoints
from shared_types.srv import CamMeta

from overhead_cv.utils.multi_robot_estimator import MultiRobotStateEstimator

from .utils.filtering_types import Measurement
from .utils.pixel_to_real import pixel_to_world


class PositionEstimator(Node):
    def __init__(self):
        super().__init__("position_estimator")

        self.prev_time = self.get_clock().now()

        # Num robots
        self.declare_parameter("N", 0)
        self.num_robots = self.get_parameter("N").value
        if not self.num_robots:
            raise ValueError("N must be specified")

        # MultiRobotStateEstimator
        self.declare_parameter("q", 0.09)
        q = self.get_parameter("q").value or 0.09
        self.declare_parameter("r", 0.005)
        r = self.get_parameter("r").value or 0.05
        self.multi_robot_estimator = MultiRobotStateEstimator(self.num_robots, q=q, r=r)

        # CamMeta
        if not self.get_cam_metadata():
            raise ValueError("Could not get camera metadata")

        # Process data
        self.unfiltered_points_sub = self.create_subscription(
            RobotPoints, "robot_observations", self.estimate_poses, 10
        )

        # `num_robots` publishers
        self._publishers = [
            self.create_publisher(Pose, f"robot{i}/pose", 10)
            for i in range(self.num_robots)
        ]

        # Calibrate
        self.declare_parameter("calibration_time", 3)
        calibration_time = self.get_parameter("calibration_time").value
        self.get_logger().info(f"Calibrating for {calibration_time} seconds")

        self.calibration_timer = self.create_timer(
            calibration_time or 3, self.stop_calibrating
        )

    def get_cam_metadata(self):
        """Gets the camera metadata from the metadata service"""
        self.meta_client = self.create_client(CamMeta, "cam_meta_data")
        while not self.meta_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("also waiting for cam node")

        future = self.get_request()
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if not response:
            return False

        self.cam_meta = {
            "focal_length": response.focal_length,
            "cam_height": response.cam_height,
            "fov": (response.fov_x, response.fov_y),
            "resolution": (response.resolution_x, response.resolution_y),
        }

        return True

    def get_request(self):
        """Sends a request to the camera metadata service"""
        req = CamMeta.Request()
        req.units = "m"  # meters
        return self.meta_client.call_async(req)

    def stop_calibrating(self):
        """Finish calibrating and assign IDs to robots"""

        self.multi_robot_estimator.assign_new_ids()
        self.calibration_timer.cancel()
        self.get_logger().info("Calibration complete. Estimating positions of robots")

        return True

    def estimate_poses(self, measured_poses: RobotPoints):
        """Update pose estimates after receiving new measurements"""
        measured_poses_world = [
            Measurement(*pixel_to_world((point.x, point.y), self.cam_meta))
            for point in measured_poses.points
        ]
        actions = {}  # TODO(sebtheiler): get the actions from PID control

        cur_time = self.get_clock().now()
        dt = (cur_time - self.prev_time).nanoseconds / 10000000
        self.prev_time = cur_time

        self.multi_robot_estimator.update_estimate(actions, measured_poses_world, dt)
        self.publish_filtered_poses()

    def publish_filtered_poses(self):
        """Publish the estimated robot positions"""
        assert self.num_robots == len(self.multi_robot_estimator.estimators)

        for i, estimator in enumerate(self.multi_robot_estimator.estimators):
            real_pos = pixel_to_world((estimator.x[0], estimator.x[1]), self.cam_meta)

            pose = Pose(
                position=Point(x=real_pos[0], y=real_pos[1]),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=0.0, w=0.0
                ),  # TODO(eugene): orientation goes here
            )

            self._publishers[i].publish(pose)


def main(args=None):
    rclpy.init(args=args)
    pos_estimator = PositionEstimator()
    rclpy.spin(pos_estimator)
    pos_estimator.destroy_node()
