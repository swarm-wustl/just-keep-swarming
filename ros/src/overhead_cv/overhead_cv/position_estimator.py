import math

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Twist

from shared_types.srv import PositionList

from rclpy.node import Node
from std_msgs.msg import Header

from overhead_cv.utils.multi_robot_estimator import MultiRobotStateEstimator
from overhead_cv.utils.quat_to_yaw import quaternion_from_yaw

from .utils.filtering_types import Command, Measurement


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

        # Process data
        self.unfiltered_points_sub = self.create_subscription(
            PoseArray, "robot_observations", self.estimate_poses, 10
        )

        # Control input
        self.twist_subs = []
        for i in range(self.num_robots):
            sub = self.create_subscription(
                Twist, f"diffdrive_twist_{i}", self.create_control_callback(i), 10
            )
            self.twist_subs.append(sub)

        self.control_input = [
            Command(lin_vel=0, ang_vel=0) for _ in range(self.num_robots)
        ]

        # `num_robots` publishers
        self._publishers = [
            self.create_publisher(PoseStamped, f"robot{i}/pose", 10)
            for i in range(self.num_robots)
        ]

        # Calibrate
        self.declare_parameter("calibration_time", 3)
        calibration_time = self.get_parameter("calibration_time").value
        self.get_logger().info(f"Calibrating for {calibration_time} seconds")

        self.calibration_timer = self.create_timer(
            calibration_time or 3, self.stop_calibrating
        )
        self.robot_service = self.create_service(
            PositionList, "get_full_robo_pos", self.get_full_robo_pos
        )

    # TODO(sebtheiler): does there need to be a timer to clear this or does the robot keep going?
    def create_control_callback(self, i: int):
        def callback(msg: Twist):
            self.control_input[i] = Command(lin_vel=msg.linear.x, ang_vel=msg.angular.z)

        return callback

    def stop_calibrating(self):
        """Finish calibrating and assign IDs to robots"""

        self.multi_robot_estimator.assign_new_ids()
        self.calibration_timer.cancel()
        self.get_logger().info("Calibration complete. Estimating positions of robots")

        return True

    def estimate_poses(self, measured_poses: PoseArray):
        """Update pose estimates after receiving new measurements"""

        measured_poses_list = [
            Measurement(
                pose.position.x,
                pose.position.y,
                2 * math.acos(pose.orientation.w),
            )
            for pose in measured_poses.poses
        ]

        cur_time = self.get_clock().now()
        dt = (cur_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = cur_time

        self.multi_robot_estimator.update_estimate(
            self.control_input, measured_poses_list, dt
        )
        self.publish_filtered_poses()

    def publish_filtered_poses(self):
        """Publish the estimated robot positions"""
        assert self.num_robots == len(self.multi_robot_estimator.estimators)

        for i, estimator in enumerate(self.multi_robot_estimator.estimators):
            theta = math.atan2(estimator.x.sin_theta, estimator.x.cos_theta)
            q = quaternion_from_yaw(theta)

            pose = PoseStamped(
                header=Header(frame_id="odom", stamp=self.get_clock().now().to_msg()),
                pose=Pose(
                    position=Point(x=estimator.x.x, y=estimator.x.y),
                    orientation=q,
                ),
            )

            self._publishers[i].publish(pose)

    def get_full_robo_pos(self, _, response):
        response.robot_n = self.num_robots
        robo_pos: Pose = PoseArray()

        pose_list = [
            Pose(
                position=Point(x=estimator.x.x, y=estimator.x.y),
                orientation=quaternion_from_yaw(
                    estimator.x.orientation
                ),  # measured_pose.orientation,
            )
            for estimator in self.multi_robot_estimator.estimators
        ]

        robo_pos.poses = pose_list
        return robo_pos


def main(args=None):
    rclpy.init(args=args)
    pos_estimator = PositionEstimator()
    rclpy.spin(pos_estimator)
    pos_estimator.destroy_node()
