#!/usr/bin/env python3
"""
preprocessing.py

Subscribes to a PointCloud2 from an OAK-D Lite camera and publishes three
filtered PointCloud2 messages:
  - /pointcloud/ground    : points near the floor plane
  - /pointcloud/obstacles : points above the ground (potential obstacles)
  - /pointcloud/gaps      : points below the ground (drop-offs / gaps)

Assumes the camera is mounted at a known height above the ground,
with the Y-axis pointing downward (DepthAI convention).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


CAMERA_HEIGHT = 0.09         
GROUND_BAND = 0.05 # ±5 cm

# anything more than OBSTACLE_MIN above the ground band is an obstacle
OBSTACLE_MIN = 0.05         

# anything below (deeper than) the ground band bottom edge is a gap
GAP_DEPTH_MARGIN = 0.05 


class PointCloudSlicer(Node):

    def __init__(self):
        super().__init__('pointcloud_slicer')

        # Declare & read parameters so they can be overridden at launch
        self.declare_parameter('camera_height', CAMERA_HEIGHT)
        self.declare_parameter('ground_band', GROUND_BAND)
        self.declare_parameter('obstacle_min', OBSTACLE_MIN)
        self.declare_parameter('gap_depth_margin', GAP_DEPTH_MARGIN)
        self.declare_parameter('input_topic', '/oak/points')

        self.camera_height  = self.get_parameter('camera_height').value
        self.ground_band    = self.get_parameter('ground_band').value
        self.obstacle_min   = self.get_parameter('obstacle_min').value
        self.gap_margin     = self.get_parameter('gap_depth_margin').value
        input_topic         = self.get_parameter('input_topic').value

        # In OAK frame, Y is down; ground plane sits at Y ≈ camera_height
        self.ground_y       = self.camera_height
        self.ground_y_min   = self.ground_y - self.ground_band
        self.ground_y_max   = self.ground_y + self.ground_band

        self.get_logger().info(
            f"Ground Y band: [{self.ground_y_min:.3f}, {self.ground_y_max:.3f}] m")

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10)

        # Publishers
        self.pub_ground    = self.create_publisher(PointCloud2, '/pointcloud/ground',    10)
        self.pub_obstacles = self.create_publisher(PointCloud2, '/pointcloud/obstacles', 10)
        self.pub_gaps      = self.create_publisher(PointCloud2, '/pointcloud/gaps',      10)

        self.get_logger().info(f"Subscribed to {input_topic}")

    # ------------------------------------------------------------------
    def callback(self, msg: PointCloud2):
        # Read all points into a numpy structured array
        # Fields expected from OAK-D Lite: x, y, z (float32)
        try:
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.array(list(gen), dtype=np.float32)   # shape (N, 3)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse PointCloud2: {e}")
            return

        if points.size == 0:
            return

        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # --- Classification masks -------------------------------------------
        # Ground: Y within the ground band
        ground_mask    = (y >= self.ground_y_min) & (y <= self.ground_y_max)

        # Obstacles: above the ground band by at least obstacle_min
        #   In OAK frame Y-down: "above" means smaller Y value
        obstacle_mask  = y < (self.ground_y_min - self.obstacle_min)

        # Gaps: below the ground band (floor missing → drop-off)
        #   "below" means larger Y value
        gap_mask       = y > (self.ground_y_max + self.gap_margin)

        self.get_logger().debug(
            f"ground={ground_mask.sum()} obstacles={obstacle_mask.sum()} gaps={gap_mask.sum()}")

        header = msg.header   # preserve frame_id and timestamp

        self.pub_ground.publish(
            self._make_cloud(header, points[ground_mask]))
        self.pub_obstacles.publish(
            self._make_cloud(header, points[obstacle_mask]))
        self.pub_gaps.publish(
            self._make_cloud(header, points[gap_mask]))

    # ------------------------------------------------------------------
    @staticmethod
    def _make_cloud(header, points: np.ndarray) -> PointCloud2:
        """Convert an (N,3) float32 array to a PointCloud2 message."""
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(header, fields, points.tolist())


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSlicer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()