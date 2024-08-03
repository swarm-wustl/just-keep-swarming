import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MultiTwistPublisher(Node):
    def __init__(self):
        super().__init__("multi_twist_publisher")

        self.declare_parameter("num_robots", 1)
        num_robots = (
            self.get_parameter("num_robots").get_parameter_value().integer_value
        )

        self._publishers = []  # Using a different attribute name to avoid conflicts
        for i in range(num_robots):
            publisher = self.create_publisher(Twist, f"/model/robot_{i}/cmd_vel", 10)
            self._publishers.append(publisher)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.i % 100 == 0:
            for index, publisher in enumerate(self._publishers):
                msg = Twist()
                msg.linear.x = random.random() / 2
                msg.angular.z = random.random() / 2
                publisher.publish(msg)
                self.get_logger().info(f"Publishing to cmd_vel_{index}: {msg}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    multi_twist_publisher = MultiTwistPublisher()
    rclpy.spin(multi_twist_publisher)
    multi_twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
