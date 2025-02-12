import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging

from rclpy.node import Node


class WebNode(Node):
    def __init__(self):
        super().__init__("web_node")
        print("node started :)")


def main(args=None):
    print("starting node :)")
    rclpy.init(args=args)

    web_node = WebNode()

    rclpy.spin(web_node)

    web_node.destroy_node()


if __name__ == "__main__":
    main()
