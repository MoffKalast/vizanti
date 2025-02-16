#!/usr/bin/env python3

import math
import rclpy
import tf2_ros

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from std_msgs.msg import Bool, Empty
from tf2_ros import TransformListener, Buffer

"""
Since navigate_through_poses is still a bit unreliable,
this node serves is a demo example of sending simple goals sequentially,
checking for range completion in between.
Goal timeouts and handling other edge cases is left as an excercise to the reader.
"""

class WaypointsToSimpleGoals(Node):
    def __init__(self):
        super().__init__('waypoints_to_simple_goals')

        self.declare_parameter('rate', 10)
        self.declare_parameter('robot_link', 'base_link')
        self.declare_parameter('goal_reached_range', 0.3)

        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.robot_link = self.get_parameter('robot_link').get_parameter_value().string_value
        self.goal_reached_range = self.get_parameter('goal_reached_range').get_parameter_value().double_value

        self.waypoints = []
        self.current_goal = None
        self.waypoints_header = None
        self.last_robot_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state_pub = self.create_publisher(Bool, '/waypoints/state', 10)
        
        self.create_subscription(PoseArray, '/waypoints', self.waypoints_callback, 10)
        self.create_subscription(Empty, '/waypoints/estop', self.estop_callback, 10)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publish_navigation_state(False)
        self.get_logger().info("Waypoints node started.")

        self.timer = self.create_timer(1.0 / self.rate, self.update)

    def parameters_callback(self, params):

        for param in params:
            if param.name == "robot_link":
                self.robot_link = param.value
            elif param.name == "goal_reached_range":
                self.goal_reached_range = param.value

        return SetParametersResult(successful=True)

    def waypoints_callback(self, msg):
        poses = list(msg.poses)
        
        if not poses:
            self.estop_callback(None)
            return

        self.waypoints = poses
        self.waypoints_header = msg.header
        self.current_goal = None
        self.get_logger().info("New path received!")

    def estop_callback(self, _):
        if self.current_goal and self.last_robot_pose and self.waypoints_header:
            goal = PoseStamped()
            goal.header = self.waypoints_header
            goal.pose = self.last_robot_pose
            self.goal_pub.publish(goal)

        self.waypoints = []
        self.current_goal = None
        self.publish_navigation_state(False)

        self.get_logger().info("Emergency stop triggered!")

    def update(self):
        if not self.waypoints:
            if self.current_goal is not None:
                self.current_goal = None
                self.publish_navigation_state(False)
            return

        if self.current_goal is None and self.waypoints:
            next_pose = self.waypoints[0]

            goal = PoseStamped()
            goal.header = self.waypoints_header
            goal.pose = next_pose
            self.goal_pub.publish(goal)

            self.current_goal = goal
            self.publish_navigation_state(True)
            self.get_logger().info(f"Published: ({next_pose.position.x},{next_pose.position.y},{next_pose.position.z}), {len(self.waypoints)} goals left.")

        if self.current_goal and self.check_goal_reached(self.current_goal):
            self.waypoints.pop(0)
            self.current_goal = None

            if not self.waypoints:
                self.get_logger().info("Path finished!")

    def check_goal_reached(self, goal):
        try:
            transform = self.tf_buffer.lookup_transform(
                goal.header.frame_id,
                self.robot_link,
                rclpy.time.Time()
            )

            pos = transform.transform.translation

            self.last_robot_pose = Pose()
            self.last_robot_pose.position.x = pos.x
            self.last_robot_pose.position.y = pos.y
            self.last_robot_pose.position.z = pos.y
            self.last_robot_pose.orientation = transform.transform.rotation

            dx = pos.x - goal.pose.position.x
            dy = pos.y - goal.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)

            return distance <= self.goal_reached_range
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def publish_navigation_state(self, state):
        self.state_pub.publish(Bool(data=state))

    def cleanup(self):
        self.waypoints = []
        self.current_goal = None
        self.publish_navigation_state(False)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsToSimpleGoals()
    rclpy.spin(node)
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()