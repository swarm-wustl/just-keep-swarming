import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging


from rclpy.node import Node

from shared_types.srv import CamMeta

from nav_msgs.msg import OccupancyGrid, MapMetaData

from std_msgs.msg import Header

from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Pose

from .pixel_to_real import pixel_to_world

# constants

ROBO_POINTS = 1
STATIC_POINTS = 2


# point in [x, y] form
# in purposes of debugging, this will use pixels and is flat
def calculate_pos(points, params):

    max_x = max(points[1][0], points[2][0])
    min_x = min(points[0][0], points[3][0])

    max_y = max(points[2][1], points[3][1])
    min_y = min(points[0][1], points[1][1])

    # This will be used to scale (x,y) in pixels to some real unit such as meters.
    # example, if the camera covers 500 by 500 meters, coversion will equal [500, 500]
    point = ((max_x - (max_x - min_x) // 2), (max_y - (max_y - min_y) // 2))

    point = pixel_to_world(pixel_coords=point, params=params)
    return point


def update_map(og_map: OccupancyGrid, point, val):
    print(int(point[0]), int(point[1]))
    print(og_map.info.width, og_map.info.height)

    if (
        point[0] >= og_map.info.width
        or point[0] < 0
        or point[1] >= og_map.info.height
        or point[1] < 0
    ):
        print("invalid point")
        return
    print(len(og_map.data))
    point = (int(point[0]), int(point[1]))
    og_map.data[int(point[1] * og_map.info.width) + int(point[0])] = val


class MapMaker(Node):
    def __init__(self):
        super().__init__("map_maker")
        self.map_meta_client = self.create_client(CamMeta, "cam_meta_data")

        self.declare_parameter("resolution", 0.05)

        map_resolution = self.get_parameter("resolution").value

        while not self.map_meta_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for cam node")

        future = self.get_request()

        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        self.map_params = {
            "focal_length": response.focal_length,
            "cam_height": response.cam_height,
            "fov": (response.fov_x, response.fov_y),
            "resolution": (response.resolution_x, response.resolution_y),
        }

        self.robot_pos_sub = self.create_subscription(
            msg_type=Float32MultiArray,
            topic="/robot_array_pos",
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

        self.og_map = OccupancyGrid()
        self.og_map.header = Header()
        self.og_map.header.frame_id = "map"

        self.og_map.data = []

        meta_data = MapMetaData()
        meta_data.resolution = map_resolution
        self.og_map.info = meta_data
        self.og_map.info.origin = Pose()
        self.og_map.info.origin.position.x = 0.0  # Map origin in meters
        self.og_map.info.origin.position.y = 0.0
        self.og_map.info.origin.position.z = 0.0
        self.og_map.info.origin.orientation.w = 1.0

        self.set_map(
            [
                response.width,
                response.height,
            ]
        )
        # from here a og_map will be generated
        # print(response)

        print("test:", pixel_to_world((0, 0), self.map_params))

    def update_robo_data(self, data_n, data, og_map: OccupancyGrid):

        # removes current robot point and remarks it in the right spot
        # print(data)
        for robo_it in range(0, data_n, 2):

            # if robo_id in robots:
            #     self.update_map(og_map, robots[robo_id], 0)

            robo_point = (data[robo_it], data[robo_it + 1])
            # print(robo_point)
            real_point = pixel_to_world(pixel_coords=robo_point, params=self.map_params)

            resoluion = self.og_map.info.resolution
            real_width = self.og_map.info.width * resoluion
            real_height = self.og_map.info.height * resoluion
            print("real:", real_point)
            print("real_W:", real_width)
            real_point = (
                (real_point[0] + real_width / 2) / resoluion,
                (real_point[1] + real_height / 2) / resoluion,
            )
            print("grid:", real_point)
            # print(real_point)
            # robo_point = calculate_pos(quad_points, self.map_params)

            update_map(og_map, real_point, 100)

            # robots[robo_id] = real_point

    def publish_map(self):

        self.og_map.header.stamp = self.get_clock().now().to_msg()
        self.pretty_print()
        self.map_publisher.publish(self.og_map)
        self.reset_map()

    def reset_map(self):
        self.og_map.data = [0] * (int(self.og_map.info.width * self.og_map.info.height))

    def pretty_print(self):
        cot = 0
        ans = ""
        for x in self.og_map.data:
            if x == 1:
                ans += "#"
            else:
                ans += "*"
            cot += 1
            if cot == self.og_map.info.width:
                print(ans)
                ans = ""
                cot = 0

    def bot_update_callback(self, data):
        # print(data)
        robot_values = data.data

        if len(self.og_map.data) == 0:
            return

        self.update_robo_data(
            data_n=len(robot_values),
            data=robot_values,
            og_map=self.og_map,
        )
        ## publish
        self.publish_map()

    def set_map(self, data):
        # gets the width and height as points, transforms, and sets that as the size of the map
        print(data)
        pixel_p = (data[0], data[1])
        real_p_end = pixel_to_world(pixel_coords=pixel_p, params=self.map_params)
        real_p_start = pixel_to_world(pixel_coords=[0, 0], params=self.map_params)

        print(real_p_end, real_p_start)
        width = real_p_end[0] - real_p_start[0] + 1
        height = real_p_end[1] - real_p_start[1] + 1
        self.og_map.info.width = int(width / self.og_map.info.resolution)
        self.og_map.info.height = int(height / self.og_map.info.resolution)

        self.og_map.data = [0] * (
            int(
                (width / self.og_map.info.resolution)
                * (height / self.og_map.info.resolution)
            )
        )
        self.og_map.info.origin.position.x = -width / 2  # Map origin in meters
        self.og_map.info.origin.position.y = -height / 2
        print(data)
        print(width)
        print(height)
        print(real_p_start)
        print(real_p_end)

    def get_request(self):
        req = CamMeta.Request()
        req.units = "m"
        return self.map_meta_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    map_maker = MapMaker()

    print("generating og_map")
    rclpy.spin(map_maker)
