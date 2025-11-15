#!/usr/bin/env python3

import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist

class GazeboLinkAttacher:
    """
    Custom implementation of link attacher for ROS 2.
    Attaches two Gazebo models by continuously updating one model's pose
    relative to another.
    """

    def __init__(self, node):
        self.node = node

        # create service clients for Gazebo
        self.get_entity_client = node.create_client(
            GetEntityState,
            '/gazebo/get_entity_state'
        )
        self.set_entity_client = node.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        # check if services are available
        if not self.get_entity_client.service_is_ready():
            self.node.get_logger().warn('get_entity_state service not yet available')
        if not self.set_entity_client.service_is_ready():
            self.node.get_logger().warn('set_entity_state service not yet available')

        # store attachment information
        self.attachments = {}  # Key: (model1, model2), Value: relative_pose

        # create a timer to update attached models
        self.update_rate = 0.01  # 100 Hz
        self.timer = node.create_timer(self.update_rate, self.update_attachments)

        self.node.get_logger().info('GazeboLinkAttacher initialized')

    def get_model_state(self, model_name, reference_frame='world'):
        """Get the current state of a model in Gazebo."""
        request = GetEntityState.Request()
        request.name = model_name
        request.reference_frame = reference_frame
        
        self.node.get_logger().info(f'Calling get_entity_state for {model_name}')
        
        future = self.get_entity_client.call_async(request)
        
        # Wait with timeout
        timeout_sec = 5.0
        start_time = self.node.get_clock().now()
        
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                break
            if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.node.get_logger().error(f'Timeout waiting for get_entity_state response for {model_name}')
                return None
        
        if future.result() is not None:
            if future.result().success:
                self.node.get_logger().info(f'Successfully got state for {model_name}')
                return future.result().state
            else:
                self.node.get_logger().error(f'get_entity_state returned success=False for {model_name}')
                return None
        else:
            self.node.get_logger().error(f'get_entity_state returned None for {model_name}')
            return None

    def set_model_state(self, model_name, pose, twist=None, reference_frame='world'):
        """
        Set the state of a model in Gazebo.

        Args:
        model_name: Name of the model
        pose: Desired pose (geometry_msgs/Pose)
        twist: Desired twist (geometry_msgs/Twist), optional
        reference_frame: Reference frame (default: 'world')

        Returns:
        bool: Success status
        """
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = model_name
        request.state.pose = pose
        request.state.reference_frame = reference_frame

        if twist is not None:
            request.state.twist = twist
        else:
            request.state.twist = Twist()  # zero velocity

        future = self.set_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.node.get_logger().error(f'Failed to set state of {model_name}')
            return False

    def compute_relative_pose(self, pose1, pose2):
        """
        Compute the relative pose of pose2 with respect to pose1.

        Args:
        pose1: Reference pose (geometry_msgs/Pose)
        pose2: Target pose (geometry_msgs/Pose)

        Returns:
        geometry_msgs/Pose: Relative pose
        """
        # simple relative position (not accounting for rotation)
        relative_pose = Pose()
        relative_pose.position.x = pose2.position.x - pose1.position.x
        relative_pose.position.y = pose2.position.y - pose1.position.y
        relative_pose.position.z = pose2.position.z - pose1.position.z

        # for simplicity, keeping the same orientation
        # TODO: compute relative quaternion
        relative_pose.orientation = pose2.orientation

        return relative_pose

    def apply_relative_pose(self, base_pose, relative_pose):
        """
        Apply a relative pose to a base pose.

        Args:
        base_pose: Base pose (geometry_msgs/Pose)
        relative_pose: Relative offset (geometry_msgs/Pose)

        Returns:
        geometry_msgs/Pose: Result pose
        """
        result_pose = Pose()
        result_pose.position.x = base_pose.position.x + relative_pose.position.x
        result_pose.position.y = base_pose.position.y + relative_pose.position.y
        result_pose.position.z = base_pose.position.z + relative_pose.position.z

        # For simplicity, using relative orientation directly
        result_pose.orientation = relative_pose.orientation

        return result_pose

    def attach(self, model1_name, model2_name):
        """
        Attach model2 to model1.

        Args:
        model1_name: Name of the base model (the one that moves freely)
        model2_name: Name of the model to attach (will follow model1)

        Returns:
        bool: Success status
        """
        # wait for services now (with timeout)
        if not self.get_entity_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('get_entity_state service not available')
            return False
        if not self.set_entity_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('set_entity_state service not available')
            return False

        # get current states of both models
        state1 = self.get_model_state(model1_name)
        state2 = self.get_model_state(model2_name)

        if state1 is None or state2 is None:
            self.node.get_logger().error(f'Failed to attach {model1_name} and {model2_name}')
            return False

        # compute and store the relative pose
        relative_pose = self.compute_relative_pose(state1.pose, state2.pose)
        attachment_key = (model1_name, model2_name)
        self.attachments[attachment_key] = relative_pose

        self.node.get_logger().info(f'Attached {model2_name} to {model1_name}')
        return True

    def detach(self, model1_name, model2_name):
        """
        Detach model2 from model1.

        Args:
        model1_name: Name of the base model
        model2_name: Name of the attached model

        Returns:
        bool: Success status
        """
        attachment_key = (model1_name, model2_name)

        if attachment_key in self.attachments:
            del self.attachments[attachment_key]
            self.node.get_logger().info(f'Detached {model2_name} from {model1_name}')
            return True
        else:
            self.node.get_logger().warn(f'No attachment found between {model1_name} and {model2_name}')
            return False

    def update_attachments(self):
        """
        Update all attached models. Called periodically by timer.
        """
        for (model1_name, model2_name), relative_pose in self.attachments.items():
            # get current state of base model
            state1 = self.get_model_state(model1_name)

            if state1 is not None:
                # compute new pose for attached model
                new_pose = self.apply_relative_pose(state1.pose, relative_pose)

                # set the attached model's pose
                self.set_model_state(model2_name, new_pose)

    def is_attached(self, model1_name, model2_name):
        """
        Check if two models are attached.

        Args:
        model1_name: Name of the base model
        model2_name: Name of the potentially attached model

        Returns:
        bool: True if attached
        """
        return (model1_name, model2_name) in self.attachments
