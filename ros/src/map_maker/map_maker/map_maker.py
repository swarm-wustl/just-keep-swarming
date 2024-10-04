import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node

from shared_types.srv import CamMeta

from std_msgs.msg import Float32MultiArray


class MapMaker(Node):
    def __init__(self):
        super().__init__("map_maker")
        self.map_meta_client = self.create_client(CamMeta, "cam_meta_data")

        while not self.map_meta_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for cam node")

        future = self.get_request()

        rclpy.spin_until_future_complete(self, future)

        response = future.result()

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

        self.robots = {}

        self.cam_data = {
            "focal_length": response.focal_length,
            "cam_height": response.cam_height,
        }
        self.map = [0]
        self.cam_data = [self.cam_data["focal_length"], self.cam_data["cam_height"]]
        self.set_map(
            [
                response.width,
                response.height,
            ]
        )
        # from here a map will be generated
        # print(response)

    def bot_update_callback(self, data):

        data_n = len(data)
        for robo_it in range(0, data_n, 9):
            robo_id = data[robo_it]

            self.robots[robo_id] = [
                (data[robo_it + 1], data[robo_it + 2]),
                (data[robo_it + 3], data[robo_it + 4]),
                (data[robo_it + 5], data[robo_it + 6]),
                (data[robo_it + 7], data[robo_it + 8]),
            ]
        ## draw in map after

    def set_map(self, data):
        print(data)
        self.map = [0] * (int(data[0] * data[1]))
        print(self.map)

    def get_request(self):
        req = CamMeta.Request()
        req.units = "m"
        return self.map_meta_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    map_maker = MapMaker()

    print("generating map")
    rclpy.spin(map_maker)
