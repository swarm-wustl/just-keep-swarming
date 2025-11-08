#!/usr/bin/env python3

# Author: Amaan Khan

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from control_algorithms.action import Dock, PID

# ros2 action send_goal /robot0/dock control_algorithms/action/Dock   "{docker_robot_id: 0, dockee_robot_id: 1, docker_face: 'front', dockee_face: 'back'}"

class DockingActionServer(Node):
    
    # define error codes
    ERROR_WRONG_ROBOT = 1
    ERROR_NO_POSITION = 2
    ERROR_DOCKEE_MOVED = 3
    ERROR_PID_FAILED = 4
    ERROR_BUSY = 5

    def __init__(self):
        super().__init__('docking_action_server')

        # declare parameters
        self.declare_parameter('robot_id')
        self.declare_parameter('robots')
        self.declare_parameter('standoff_distance', 0.02)
        self.declare_parameter('dockee_movement_threshold', 0.05)

        # get params
        self.robot_id = self.get_parameter('robot_id').value
        robot_ids = self.get_parameter('robots').value
        self.standoff_distance = self.get_parameter('standoff_distance').value
        self.dockee_movement_threshold = self.get_parameter('dockee_movement_threshold').value

        self.robot_params = {
            'LENGTH': 0.08, # m
            'WIDTH': 0.065 # m
        }

        # calcluate face offsets from center
        half_length = self.robot_params['LENGTH'] / 2.0
        half_width = self.robot_params['WIDTH'] / 2.0

        self.FACE_OFFSETS = {
            'front': {'x': half_length, 'y': 0.0, 'yaw': 0.0},
            'back': {'x': -half_length, 'y': 0.0, 'yaw': math.pi},
            'left': {'x': 0.0, 'y': half_width, 'yaw': math.pi / 2},
            'right': {'x': 0.0, 'y': -half_width, 'yaw': -math.pi / 2}
        }

        # state tracking
        self.robot_positions = {} # {robot_id: PoseStamped}
        self.robot_subscriptions = {}
        self.active_dockee = None
        self.initial_dockee_pose = None
        self.current_goal_handle = None
        self.pid_goal_handle = None
        self.target_pose = None

        # subscribe to all known robot poses
        for robot_id in robot_ids:
            topic = f'/model/robot{robot_id}/pose'
            self.robot_subscriptions[robot_id] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, rid=robot_id: self.pose_callback(rid, msg),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')

        
        # create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # create pid action client
        self.pid_client = ActionClient(
            self,
            PID,
            f'/pid',
            callback_group=self.callback_group
        )

        # create docking action server
        self.docking_server = ActionServer(
            self,
            Dock,
            f'/robot{self.robot_id}/dock',
            self.execute_docking,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(f'Docking Action Server initialized for robot{self.robot_id}')
        
    
    def pose_callback(self, robot_id, msg):
        """cache the latest pose for each robot"""
        self.robot_positions[robot_id] = msg


    def goal_callback(self, goal_request):
        """accept or reject docking goal"""
        self.get_logger().info(
            f'Received docking request: robot{goal_request.docker_robot_id} '
            f'({goal_request.docker_face}) -> robot {goal_request.dockee_robot_id} '
            f'({goal_request.dockee_face})'
        )

        self.get_logger().info(f'Currently tracking {len(self.robot_positions)} robots: {list(self.robot_positions.keys())}')

        # check if this is the correct robot
        if goal_request.docker_robot_id != self.robot_id:
            self.get_logger().warn(
                f'Goal rejected: docker_robot_id={goal_request.docker_robot_id} '
                f'but this is robot {self.robot_id}'
            )
            return GoalResponse.REJECT
        
        # check if already busy
        if self.active_dockee is not None:
            self.get_logger().warn(f'Goal rejected: already docking with robot {self.active_dockee}')
            return GoalResponse.REJECT
        
        # check if dockee position is known
        if goal_request.dockee_robot_id not in self.robot_positions:
            self.get_logger().warn(f'Goal rejected: no position data for robot {goal_request.dockee_robot_id}')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    

    def cancel_callback(self, goal_handle):
        """handles cancellation request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    
    async def execute_docking(self, goal_handle):
        """MAIN LOGIC"""
        self.get_logger().info('Executing docking goal...')
        self.current_goal_handle = goal_handle
        goal = goal_handle.request

        try:
            # mark dockee as active
            self.active_dockee = goal.dockee_robot_id

            # store initial dockee position for movement detection
            self.initial_dockee_pose = self.robot_positions[goal.dockee_robot_id].pose

            # PHASE 1: calculate docking pose
            self.publish_feedback('calculating')

            target_pose = self.calculate_docking_pose(
                self.initial_dockee_pose,
                goal.dockee_face,
                goal.docker_face
            )

            self.target_pose = target_pose

            self.get_logger().info(
                f'Calculated target pose: ({target_pose.position.x:.3f}, '
                f'{target_pose.position.y:.3f}), '
                f'yaw={self.get_yaw_from_quaternion(target_pose.orientation):.2f}rad'
            )

            # PHASE 2: send goal to pid controller
            self.publish_feedback('approaching')

            if not self.pid_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('PID action server not available')
                goal_handle.abort()
                return Dock.Result(success=False, error_code=self.ERROR_PID_FAILED)
            
            # create pid goal
            pid_goal = PID.Goal()
            pid_goal.header.stamp = self.get_clock().now().to_msg()
            pid_goal.header.frame_id = 'map'
            pid_goal.target_pose = target_pose
            pid_goal.robot_id = self.robot_id

            self.get_logger().info('Sending goal to PID controller...')

            # send goal and wait for result
            pid_goal_future = self.pid_client.send_goal_async(
                pid_goal,
                feedback_callback=self.pid_feedback_callback
            )

            # wait for acceptance
            pid_goal_handle = await pid_goal_future

            if not pid_goal_handle.accepted:
                self.get_logger().error('PID goal rejected')
                goal_handle.abort()
                return Dock.Result(success=False, error_code=self.ERROR_PID_FAILED)
            
            self.pid_goal_handle = pid_goal_handle
            self.get_logger().info('PID goal accepted, waiting for result...')

            # wait for completion
            pid_result_future = pid_goal_handle.get_result_async()
            pid_result = await pid_result_future

            # check result
            if pid_result.result.error_code != 0:
                self.get_logger().error(f'PID action failed with error code: {pid_result.result.error_code}')
                goal_handle.abort()
                return Dock.Result(success=False, error_code=self.ERROR_PID_FAILED)
            
            # success!
            self.get_logger().info('Docking completed successfully')
            goal_handle.succeed()
            return Dock.Result(success=True, error_code=0)
        
        except Exception as e:
            self.get_logger().error(f'Docking failed with exception: {e}')
            goal_handle.abort()
            return Dock.Result(success=False, error_code=self.ERROR_PID_FAILED)
        
        finally:
            # clean up
            self.active_dockee = None
            self.initial_dockee_pose = None
            self.current_goal_handle = None
            self.pid_goal_handle = None


    def pid_feedback_callback(self, msg):
        """handles feedback from pid controller and checks for dockee movement"""
        if self.current_goal_handle is None:
            return 
        
        current_pose = msg.feedback.current_pose

        # check if dockee movced
        if self.active_dockee in self.robot_positions:
            current_dockee_pose = self.robot_positions[self.active_dockee].pose
            distance_moved = self.calculate_distance(self.initial_dockee_pose.position, current_dockee_pose.position)

            if distance_moved > self.dockee_movement_threshold:
                self.get_logger().warn(f'Dockee moved {distance_moved:.3f}m during docking! Aborting.')

                # cancel pid goal
                if self.pid_goal_handle is not None:
                    self.pid_goal_handle.cancel_goal_async()

                # abort docking
                self.current_goal_handle.abort()
                return
        
        # publish docking feedback
        self.publish_feedback('approaching')
        
    
    def publish_feedback(self, phase):
        """publish docking action feedback"""
        if self.current_goal_handle is None:
            return
        
        feedback = Dock.Feedback()
        feedback.current_phase = phase
        
        self.current_goal_handle.publish_feedback(feedback)
    
    def calculate_docking_pose(self, dockee_pose, dockee_face, docker_face):
        """
        calculate target pose for docker robot to align with dockee robot.
        
        args:
            dockee_pose: current Pose of the dockee robot
            dockee_face: face of dockee to dock to
            docker_face: face of docker that will make contact
            
        returns:
            pose: tTarget pose for docker's base_link
        """
        dockee_yaw = self.get_yaw_from_quaternion(dockee_pose.orientation)
        
        dockee_face_config = self.FACE_OFFSETS[dockee_face]
        docker_face_config = self.FACE_OFFSETS[docker_face]
        
        # calculate dockee face position in world frame
        cos_yaw = math.cos(dockee_yaw)
        sin_yaw = math.sin(dockee_yaw)
        
        dockee_face_x = dockee_pose.position.x + \
                        (dockee_face_config['x'] * cos_yaw - 
                         dockee_face_config['y'] * sin_yaw)
        
        dockee_face_y = dockee_pose.position.y + \
                        (dockee_face_config['x'] * sin_yaw + 
                         dockee_face_config['y'] * cos_yaw)
        
        # calculate outward normal direction of dockee face
        dockee_face_normal_yaw = dockee_yaw + dockee_face_config['yaw']
        
        # docker orientation (face the dockee)
        docker_target_yaw = dockee_face_normal_yaw + math.pi
        docker_target_yaw = math.atan2(math.sin(docker_target_yaw), 
                                       math.cos(docker_target_yaw))
        
        # calculate separation distance
        dockee_face_dist = math.sqrt(dockee_face_config['x']**2 + 
                                     dockee_face_config['y']**2)
        docker_face_dist = math.sqrt(docker_face_config['x']**2 + 
                                     docker_face_config['y']**2)
        total_separation = dockee_face_dist + self.standoff_distance + docker_face_dist
        
        # calculate target position
        target_pose = Pose()
        target_pose.position.x = dockee_face_x - total_separation * math.cos(dockee_face_normal_yaw)
        target_pose.position.y = dockee_face_y - total_separation * math.sin(dockee_face_normal_yaw)
        target_pose.position.z = dockee_pose.position.z
        target_pose.orientation = self.yaw_to_quaternion(docker_target_yaw)
        
        return target_pose
    
    
    def get_yaw_from_quaternion(self, quat):
        """extract yaw angle from quaternion"""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    

    def yaw_to_quaternion(self, yaw):
        """convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    

    def calculate_distance(self, pos1, pos2):
        """calculate Euclidean distance between two positions"""
        return math.sqrt((pos1.x - pos2.x)**2 + 
                        (pos1.y - pos2.y)**2 + 
                        (pos1.z - pos2.z)**2)


def main(args=None):
    rclpy.init(args=args)

    docking_action_server = DockingActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(docking_action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        docking_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()