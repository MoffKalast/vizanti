#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav_msgs.msg import Path
from nav2_msgs.action import NavigateThroughPoses

class PathToNavigateThroughPosesNode(Node):

    def __init__(self):
        super().__init__('path_to_navigate_through_poses')
        self.subscription = self.create_subscription(
            Path,
            '/navigate_through_path',
            self.path_callback,
            10
        )
        
        self.action_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for /navigate_through_poses action server...')
            
    def path_callback(self, msg):
        self.get_logger().info('Received a path with %d poses' % len(msg.poses))

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = msg.poses
        
        self.send_goal(goal_msg)

    def send_goal(self, goal):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('/navigate_through_poses action server not available!')
            return
        
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('NavigateThroughPoses goal rejected :(')
            return

        self.get_logger().info('NavigateThroughPoses goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Finished NavigateThroughPoses with result: %s' % result)


def main(args=None):
    rclpy.init(args=args)

    node = PathToNavigateThroughPosesNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()