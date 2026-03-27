import rclpy
from rclpy.node import Node
import message_filters
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

class CloudToGrid(Node):
    def __init__(self):
        super().__init__('CloudToGrid')

        # self.robots = self.create_subscription(
        #     PoseArray, "robot_observations", self.estimate_poses, 10
        # )

        self.resolution = 0.05
        self.map_size = 200

        # These do not have callbacks; they feed into the TimeSynchronizer
        self.obs_sub = message_filters.Subscriber(self, PointCloud2, '/terrain_slices/obstacles')
        self.floor_sub = message_filters.Subscriber(self, PointCloud2, '/terrain_slices/floor')
        self.gap_sub = message_filters.Subscriber(self, PointCloud2, '/terrain_slices/gaps')

        # Synchronize based on header timestamps (queue=10, slop=0.05 seconds)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.obs_sub, self.floor_sub, self.gap_sub], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.sync_paint_callback)

        self.grid_pub = self.create_publisher(OccupancyGrid, '/terrain_grid', 10)
        self.get_logger().info("CloudToGrid initialized.")

    def pointcloud_to_grid_coords(self, msg):
        """Extracts X, Y from PointCloud2 and converts to 2D grid indices."""
        points_gen = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        points_list = list(points_gen)

        pts = np.array(points_list)

        # downsample to grid resolution
        grid_x = np.round(pts[:, 0] / self.resolution).astype(int)
        grid_y = np.round(pts[:, 1] / self.resolution).astype(int)

        in_bound = (grid_x >= 0) & (grid_x < self.map_size) & (grid_y >= 0) & (grid_y < self.map_size)
        
        return grid_x[in_bound], grid_y[in_bound]


    # def robo_occupancy_filter(self, msg):

    

    def sync_paint_callback(self, obs_msg, floor_msg, gap_msg):
        """Fires only when all 3 slices from the same camera frame arrive."""
        grid = np.full((self.map_size, self.map_size), -1, dtype=np.int8)

        fx, fy = self.pointcloud_to_grid_coords(floor_msg)
        gx, gy = self.pointcloud_to_grid_coords(gap_msg)
        ox, oy = self.pointcloud_to_grid_coords(obs_msg)

        if len(fx) > 0: grid[fy, fx] = 0    # Floor
        if len(gx) > 0: grid[gy, gx] = 50   # Gaps
        if len(ox) > 0: grid[oy, ox] = 100  # Obstacles

        grid_msg = OccupancyGrid()
        grid_msg.header = obs_msg.header
        
        grid_msg.info = MapMetaData()
        grid_msg.info.resolution = float(self.resolution)
        grid_msg.info.width = self.map_size
        grid_msg.info.height = self.map_size
        
        grid_msg.data = grid.flatten().tolist()

        self.grid_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    cloud_grid = CloudToGrid()
    rclpy.spin(cloud_grid)
    cloud_grid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()