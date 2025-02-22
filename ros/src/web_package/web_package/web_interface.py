import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging
import threading
from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
from flask_socketio import SocketIO
from std_msgs.msg import String

from rclpy.node import Node


class WebNode(Node):
    def __init__(self):
        super().__init__("web_node")
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        print("node started: ")

        self.app = Flask(__name__)
        CORS(self.app)
        self.setup_routes()
        SocketIO(self.app).run(self.app)

    def setup_routes(self):  
        @self.app.route('/')
        def index():
            msg = String()
            msg.data = "hello"
            self.publisher_.publish(msg)
            print("test")
            return render_template('index.html')
    
def start_web_node():
    rclpy.init()
    web_node = WebNode()
    rclpy.spin(web_node)
    web_node.destroy_node()
    print("stopped")

def main():
    rclpy.init() 
    web_node = WebNode()
    rclpy.spin(web_node)
    print("ran")
    
    
    # ros_thread = threading.Thread(target=start_web_node, daemon=True)
    # ros_thread.start()
    #SocketIO(app).run(app)

if __name__ == "__main__":
    main()