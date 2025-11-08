#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_algorithms.action import Dock


class DockingTestClient(Node):
    def __init__(self):
        super().__init__('docking_test_client')
        
        self.declare_parameter('docker_id', 0)
        self.declare_parameter('dockee_id', 1)
        self.declare_parameter('docker_face', 'front')
        self.declare_parameter('dockee_face', 'back')
        
        docker_id = self.get_parameter('docker_id').value
        
        self.client = ActionClient(self, Dock, f'/robot{docker_id}/dock')
        
    def send_goal(self, docker_id, dockee_id, docker_face, dockee_face):
        self.get_logger().info('Waiting for docking action server...')
        self.client.wait_for_server()
        
        goal = Dock.Goal()
        goal.docker_robot_id = docker_id
        goal.dockee_robot_id = dockee_id
        goal.docker_face = docker_face
        goal.dockee_face = dockee_face
        
        self.get_logger().info(
            f'Sending docking goal: robot {docker_id} ({docker_face}) -> '
            f'robot {dockee_id} ({dockee_face})')
        
        self.goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self.goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted')
        
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback - Phase: {feedback.current_phase}, '
            f'Distance: {feedback.distance_to_target:.3f}m')
    
    def result_callback(self, future):
        result = future.result().result
        
        if result.success:
            self.get_logger().info('Docking succeeded!')
        else:
            self.get_logger().error(
                f'Docking failed with error code: {result.error_code}')


def main(args=None):
    rclpy.init(args=args)
    
    client = DockingTestClient()
    
    docker_id = client.get_parameter('docker_id').value
    dockee_id = client.get_parameter('dockee_id').value
    docker_face = client.get_parameter('docker_face').value
    dockee_face = client.get_parameter('dockee_face').value
    
    client.send_goal(docker_id, dockee_id, docker_face, dockee_face)
    
    rclpy.spin(client)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()