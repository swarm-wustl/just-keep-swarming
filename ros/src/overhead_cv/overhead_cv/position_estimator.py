import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray
import rclpy

# pylint: disable=import-error
from shared_types.srv import CamMeta

from overhead_cv.utils.pose_convert import create_pose, create_quat, create_point
from overhead_cv.utils.multi_robot_estimator import MultiRobotStateEstimator

from .pixel_to_real import pixel_to_world


class PositionEstimator(Node):
    def __init__(self):
        super().__init__("position_estimator")

        self.declare_parameter("q", 0.09)
        q = self.get_parameter("q").value
        self.declare_parameter("r", 0.005)
        r = self.get_parameter("r").value

        # MAKE THIS NOT HARDCODED
        self.num_robots = 4

        self.meta_client = self.create_client(CamMeta, "cam_meta_estimator")

        while not self.meta_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("also waiting for cam node")

        future = self.get_request()

        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        self.cam_meta = {
            "focal_length": response.focal_length,
            "cam_height": response.cam_height,
            "fov": (response.fov_x, response.fov_y),
            "resolution": (response.resolution_x, response.resolution_y),
        }
        self.raw_poses_sub = self.create_subscription(
            Float32MultiArray, "robot_array_pos", self.estimate_poses, 10
        )

        # this has been changed from floatarray to posearray
        self.filtered_poses_pub = self.create_publisher(
            PoseArray, "filtered_robot_array_pos", 10
        )

        self.multi_robot_estimator = MultiRobotStateEstimator(q=q, r=r)

    def get_request(self):
        req = CamMeta.Request()
        req.units = "m"
        return self.meta_client.call_async(req)

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
        data = [Pose()] * self.num_robots
        print("filtered")
        for estimator_id, estimator in self.multi_robot_estimator.estimators.items():
            if estimator.num_estimates_received < 20:
                continue

            real_pos = pixel_to_world(estimator.x, self.cam_meta)

            # quanternion is open
            pose = create_pose(
                create_point(real_pos[0], real_pos[1], 0), create_quat(0, 0, 0, 0)
            )

            # TODO(seb): make the estimator ids replace a lost id if its lost, use a queue or smth # pylint: disable=fixme
            data[estimator_id] = pose
            # data.append(real_pos[0])
            # data.append(real_pos[1])
            # data.append(float(estimator_id))
            print((estimator_id, estimator.x))

        filtered_poses = PoseArray()
        filtered_poses.poses = data
        self.filtered_poses_pub.publish(filtered_poses)


def main(args=None):
    rclpy.init(args=args)

    pos_estimator = PositionEstimator()

    print("Estimating positions of robots")

    rclpy.spin(pos_estimator)

    pos_estimator.destroy_node()
