import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor
from planning.kinematic_graph import parse_topology_text, unfold_graph
from planning.prompts import STRUCTURE_BUILDER_PROMPT
from shared_types.action import BuildStructure, Navigate
import os

class StructureBuilder(Node):
    def __init__(self):
        super().__init__('structure_builder')
        
        # Callback group allows the client call inside the server callback
        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            BuildStructure,
            'build_structure',
            self.execute_callback,
            callback_group=self._cb_group
        )
        
        # Navigate client
        self._nav_client = ActionClient(
            self, 
            Navigate, 
            '/navigate', 
            callback_group=self._cb_group
        )

        self.get_logger().info('Structure Builder Action Server is ready.')

        # Mock OpenAI Key setup
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            self.get_logger().warn("OPENAI_API_KEY env var not set. LLM calls will fail or need mocking.")

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        description = goal_handle.request.description
        
        # Query LLM
        self.get_logger().info(f'Prompting GPT-4o for: "{description}"')
        try:
            generated_topology = self.call_llm(description)
            self.get_logger().info(f'LLM Output:\n{generated_topology}')
        except Exception as e:
            self.get_logger().error(f"LLM Call failed: {e}")
            goal_handle.abort()
            result = BuildStructure.Result()
            result.success = False
            result.message = f"LLM Error: {str(e)}"
            return result

        # Parse and Unfold
        try:
            graph = parse_topology_text(generated_topology)
            # TODO: way to specify nonzero origin
            target_poses_map = unfold_graph(graph, root_id=0)
        except Exception as e:
            self.get_logger().error(f"Parsing failed: {e}")
            goal_handle.abort()
            result = BuildStructure.Result()
            result.success = False
            result.message = f"Parsing Error: {str(e)}"
            return result
        
        # Convert map to lists for the Action Goal
        # only send commands for robots present in the graph
        # (robots not mentioned stay stationary)
        # TODO: maybe they should move out of the way?
        robot_ids = list(target_poses_map.keys())
        target_locations = list(target_poses_map.values())
        
        self.get_logger().info(f"Generated {len(robot_ids)} target poses.")

        # Send to Navigate Action Server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigate action server not available!')
            goal_handle.abort()
            return BuildStructure.Result(success=False, message="Nav Server unavailable")

        nav_goal = Navigate.Goal()
        nav_goal.robot_ids = robot_ids
        nav_goal.target_locations = target_locations
        # self.get_logger().info(f"Target locations: {target_locations}")

        self.get_logger().info("Sending goal to /navigate...")
        send_goal_future = self._nav_client.send_goal_async(nav_goal)
        
        # Await the goal acceptance
        goal_handle_nav = await send_goal_future
        if not goal_handle_nav.accepted:
            self.get_logger().error('Navigation goal rejected')
            goal_handle.abort()
            return BuildStructure.Result(success=False, message="Nav Goal rejected")

        # Await the result
        get_result_future = goal_handle_nav.get_result_async()
        nav_result = await get_result_future
        
        # Success
        goal_handle.succeed()
        result = BuildStructure.Result()
        result.success = True
        result.message = "Structure built successfully."
        return result

    def call_llm(self, user_description: str) -> str:
        """
        Interacts with OpenAI API.
        This requires `pip install openai` and `OPENAI_API_KEY` env var.
        """
        # If no key, return a hardcoded mock for testing safety
        if not self.openai_api_key:
            self.get_logger().warn("Using MOCK LLM response (No API Key).")
            # Mocking a "line" of robots
            # return """
            # 0.front 1.back
            # 1.front 2.right
            # 2.front 3.back
            # 3.front 4.right
            # 4.front 5.back
            # 5.front 6.right
            # 6.front 7.back
            # 7.front 8.right
            # 8.front 9.back
            # """

            return """
        1.back 2.front
        1.left 3.right
        1.right 4.left
        1.front 5.back
        5.left 6.right
        5.right 7.left
        5.front 8.back
        8.left 9.right
        8.right 10.left
            """

        from openai import OpenAI
        client = OpenAI(api_key=self.openai_api_key)

        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": STRUCTURE_BUILDER_PROMPT},
                {"role": "user", "content": user_description}
            ],
            temperature=0.2
        )
        return response.choices[0].message.content.strip()

def main(args=None):
    rclpy.init(args=args)
    structure_builder = StructureBuilder()
    
    executor = MultiThreadedExecutor()
    
    rclpy.spin(structure_builder, executor=executor)
    
    structure_builder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
