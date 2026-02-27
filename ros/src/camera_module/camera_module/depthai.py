import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import numpy as np

class SpatialProjector(Node):
    def __init__(self):
        super().__init__('spatial_projector')
        self.bridge = CvBridge()
        self.intrinsics = None

        # 1. Subscribe to Camera Info once to get focal lengths
        self.info_sub = self.create_subscription(CameraInfo, '/oak/rgb/camera_info', self.info_callback, 10)

        # 2. Setup Synchronized Subscribers for RGB and Depth
        self.rgb_sub = message_filters.Subscriber(self, Image, '/oak/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/oak/stereo/image_raw')

        # This syncs them based on their header timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

    def info_callback(self, msg):
        # Extract intrinsic matrix parameters: fx, fy, cx, cy
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.intrinsics = msg.k
        self.get_logger().info("Camera Intrinsics Received")

    def sync_callback(self, rgb_msg, depth_msg):
        if self.intrinsics is None: return

        # Convert ROS Image to OpenCV/Numpy
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        
        # --- YOUR AI LOGIC HERE ---
        # Example: Let's say you found a target at pixel (u, v)
        u, v = 320, 240 
        
        # Get depth at that pixel (usually in mm for OAK-D)
        z_mm = depth_frame[v, u]
        
        if z_mm > 0:
            z = z_mm / 1000.0 # Convert to meters
            # Apply Projection Equations
            # x = (u - cx) * z / fx
            x = (u - self.intrinsics[2]) * z / self.intrinsics[0]
            y = (v - self.intrinsics[5]) * z / self.intrinsics[4]
            
            self.get_logger().info(f"Object detected at: {x:.2f}m, {y:.2f}m, {z:.2f}m")

def main():
    rclpy.init()
    node = SpatialProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()