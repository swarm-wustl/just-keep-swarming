import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import rclpy


from overhead_cv.utils.multi_robot_estimator import MultiRobotStateEstimator


class PositionEstimator(Node):
    def __init__(self):
        super().__init__("position_estimator")

        self.declare_parameter("q", 0.09)
        q = self.get_parameter("q").value
        self.declare_parameter("r", 0.005)
        r = self.get_parameter("r").value

        self.raw_poses_sub = self.create_subscription(
            Float32MultiArray, "robot_array_pos", self.estimate_poses, 10
        )
        self.filtered_poses_pub = self.create_publisher(
            Float32MultiArray, "filtered_robot_array_pos", 10
        )

        self.multi_robot_estimator = MultiRobotStateEstimator(q=q, r=r)

    def estimate_poses(self, poses: Float32MultiArray):
        print("unfiltered data")
        print(poses.data)
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
            dt=0.05,  # TODO(sebtheiler): make this dynamic # pylint: disable=fixme
        )

        self.publish_filtered_poses()

    def publish_filtered_poses(self):
        data = []
        print("filtered")
        for estimator_id, estimator in self.multi_robot_estimator.estimators.items():
            if estimator.num_estimates_received < 20:
                continue

            data.append(estimator.x[0])
            data.append(estimator.x[1])
            data.append(float(estimator_id))
            print((estimator_id, estimator.x))

        filtered_poses = Float32MultiArray()
        filtered_poses.data = data
        self.filtered_poses_pub.publish(filtered_poses)


def main(args=None):
    rclpy.init(args=args)

    pos_estimator = PositionEstimator()

    print("Estimating positions of robots")

    rclpy.spin(pos_estimator)

    pos_estimator.destroy_node()
