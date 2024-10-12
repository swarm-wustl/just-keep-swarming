import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from shared_types.srv import CamMeta

from nav_msgs.msg import OccupancyGrid, MapMetaData

from std_msgs.msg import Float32MultiArray

# constants

ROBO_POINTS = 1
STATIC_POINTS = 2


# in purposes of debugging, this will use pixels and is flat
def calculate_pos(points):

    max_x = max(points[1][0], points[2][0])
    min_x = min(points[0][0], points[3][0])

    max_y = max(points[2][1], points[3][1])
    min_y = min(points[0][1], points[1][1])

    # This will be used to scale (x,y) in pixels to some real unit such as meters.
    # example, if the camera covers 500 by 500 meters, coversion will equal [500, 500]
    point = ((max_x - (max_x - min_x) // 2), (max_y - (max_y - min_y) // 2))

    # projection will occur here

    return point


def update_map(og_map: OccupancyGrid, point, val):
    if point[0] >= og_map.info.width or point[1] >= og_map.info.height:
        print("invalid point")
        return
    og_map.data[point[1] * og_map.info.width + point[0]] = val


def update_robo_data(data_n, data, og_map: OccupancyGrid, robots):

    for robo_it in range(0, data_n, 9):
        robo_id = data[robo_it]

        if robo_id in robots:
            update_map(og_map, robots[robo_id], 0)

        quad_points = [
            (data[robo_it + 1], data[robo_it + 2]),
            (data[robo_it + 3], data[robo_it + 4]),
            (data[robo_it + 5], data[robo_it + 6]),
            (data[robo_it + 7], data[robo_it + 8]),
        ]

        robo_point = calculate_pos(quad_points)

        update_map(og_map, robo_point, 1)

        robots[robo_id] = robo_point


class MapMaker(Node):
    def __init__(self):
        super().__init__("map_maker")
        self.map_meta_client = self.create_client(CamMeta, "cam_meta_data")

        self.declare_parameter("resolution", 0.5)

        resolution = self.get_parameter("resolution").value

        while not self.map_meta_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for cam node")

        future = self.get_request()

        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        self.set_map(
            [
                response.width,
                response.height,
            ]
        )
        self.robot_pos_sub = self.create_subscription(
            msg_type=Float32MultiArray,
            topic="/multi_array_pos",
            callback=self.bot_update_callback,
            qos_profile=10,
        )

        self.map_size_update = self.create_subscription(
            msg_type=Float32MultiArray,
            topic="/img_shape",
            callback=self.set_map,
            qos_profile=10,
        )

        self.map_publisher = self.create_publisher(OccupancyGrid, "/og_map", 10)

        self.robots = {}

        self.cam_data = {
            "focal_length": response.focal_length,
            "cam_height": response.cam_height,
        }
        self.og_map = OccupancyGrid
        self.og_map.data = []
        self.cam_data = [self.cam_data["focal_length"], self.cam_data["cam_height"]]

        meta_data = MapMetaData()
        meta_data.resolution = resolution
        self.og_map.info = meta_data
        # from here a og_map will be generated
        # print(response)

    # point in [x, y] form

    def publish_map(self):

        self.og_map.info.map_load_time = self.get_clock().now()
        self.map_publisher.publish(self.og_map)

    def bot_update_callback(self, data):

        update_robo_data(
            data_n=len(data), data=data, og_map=self.og_map, robots=self.robots
        )
        ## publish
        self.publish_map()

    def set_map(self, data):
        # projection will be called here once completed
        self.og_map.data = [0] * (int(data[0] * data[1]))
        self.og_map.info.width = data[0]
        self.og_map.info.height = data[1]
        print(self.og_map)

    def get_request(self):
        req = CamMeta.Request()
        req.units = "m"
        return self.map_meta_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    map_maker = MapMaker()

    print("generating og_map")
    rclpy.spin(map_maker)
