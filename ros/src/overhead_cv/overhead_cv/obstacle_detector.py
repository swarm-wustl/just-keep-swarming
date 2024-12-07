import math

import cv2

import numpy as np

import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import Float32MultiArray

import numpy as np

class obstacles(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter("grid_dim", (10,10)) #grid dimensions
        self.occupancy_height, self.occupancy_width = self.get_parameter("grid_dim").value

        self.cv_bridge = CvBridge()

        self.poses_emit = self.create_publisher(
            Float32MultiArray, "obstacles", 10
        )

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/video",
            callback=self.obstacle_detector_callback,
            qos_profile=10,
        )

    def emit_obstacles(self, num_obstacles, obstacles):
        if num_obstacles == 0:
            return
        
        msg = Float32MultiArray()
        msg.data = [float(val) for row in obstacles for val in row]
        self.poses_emit.publish((msg))

    def occupancy_to_pixel(self,x,y,frame_width,frame_height):
        occupancy_square_width = math.ceil(frame_width/self.occupancy_width)
        occupancy_square_height= math.ceil(frame_height/self.occupancy_height)

        x1 = x * occupancy_square_width
        if x==self.occupancy_width:
            x2=frame_width
        else:
            x2 = (x+1) * occupancy_square_width

        y1 = y * occupancy_square_height
        if y==self.occupancy_height:
            y2=frame_width
        else:
            y2 = (y+1) * occupancy_square_height

        return x1,x2,y1,y2

    def obstacle_detector_callback(self, data):
        frame = self.cv_bridge.imgmsg_to_cv2(data)

        frame_width = len(frame[0])

        frame_height = len(frame)

        range_=12

        background = np.median(frame)

        occupancy_grid=[self.occupancy_width,self.occupancy_height]

        num_obstacles=0

        for x in range(0, occupancy_grid.shape[0]):
            for y in range(0, occupancy_grid.shape[1]):
                x1,x2,y1,y2 = self.occupancy_to_pixel(x,y,frame_width,frame_height)
                if frame[x1:x2,y1:y2].mean > (range_+background):
                    occupancy_grid[x][y]=1
                    num_obstacles+=1
                else:
                    occupancy_grid[x][y]=0
        
        self.emit_obstacles(num_obstacles, occupancy_grid)

def main(args=None):
    print("starting obstacle detector")

    rclpy.init(args=args)

    obstacle = obstacles()

    rclpy.spin(obstacle)