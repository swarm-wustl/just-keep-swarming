import numpy as np
from rclpy import Node
from std_msgs.msg import Float32MultiArray

from overhead_cv.utils.multi_robot_estimator import MultiRobotStateEstimator


class PositionEstimator(Node):
    def __init__(self):
        super().__init__("position_estimator")

        self.raw_poses_sub = self.create_subscription(
            Float32MultiArray, "multi_array_pos", self.estimate_poses, 10
        )
        self.filtered_poses_pub = self.create_publisher(
            Float32MultiArray, "filtered_array_pos", 10
        )

        self.multi_robot_estimator = MultiRobotStateEstimator()

    def estimate_poses(self, poses: Float32MultiArray):
        data = poses.data
        if len(data) % 2 != 0:
            self.get_logger().error("Received odd number of poses (not in pairs)")
            return

        N = len(data) // 2
        zs = []
        for i in range(N):
            raw_x = data[i * 2]
            raw_y = data[i * 2 + 1]
            zs.append(
                np.array([raw_x, raw_y, 0])
            )  # TODO(sebtheiler): add theta # pylint: disable=fixme

        id2u = {}
        self.multi_robot_estimator.update_estimate(
            id2u,
            zs,
        )

        self.publish_filtered_poses()

    def publish_filtered_poses(self):
        data = []
        for estimator_id, estimator in self.multi_robot_estimator.estimators.values():
            data.append(estimator.x[0])
            data.append(estimator.x[1])
            data.append(estimator_id)

        filtered_poses = Float32MultiArray()
        filtered_poses.data = data
        self.filtered_poses_pub.publish(filtered_poses)
