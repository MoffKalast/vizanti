#!/usr/bin/env python3

import math
import rclpy
import tf2_ros

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from std_msgs.msg import Bool, Empty
from tf2_ros import TransformListener, Buffer

"""
Since navigate_through_poses is still a bit unreliable, this node serves is a demo example of sending simple goals sequentially, checking for range completion in between. Also usable for other nav stacks other than nav2.

Paths received on /waypoints will be sent one by one to /goal_pose, once the robot arrives at each, the next one will be sent.
The /waypoints/state bool topic publishes true if we're currently navigating a path.
If a /waypoints/estop is received or a preempted goal is sent to /goal_pose by some other node, we abort.

Goal timeouts and handling other edge cases is left as an excercise to the reader.
"""

def pose_equals(a, b):
    pa, pb = a.pose.position, b.pose.position
    oa, ob = a.pose.orientation, b.pose.orientation

    return (
        pa.x == pb.x and
        pa.y == pb.y and 
        pa.z == pb.z and 
        oa.x == ob.x and 
        oa.y == ob.y and 
        oa.z == ob.z and 
        oa.w == ob.w
    )

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

        self.tf_buffer = None
        self.tf_listener = None
        self.timer = None

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state_pub = self.create_publisher(Bool, '/waypoints/state', 10)

        self.create_subscription(PoseArray, '/waypoints', self.waypoints_callback, 10)
        self.create_subscription(Empty, '/waypoints/estop', self.estop_callback, 10)

        # Check for any other goals being sent, abort on preempt
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.state_pub.publish(Bool(data=False))
        self.get_logger().info("Waypoints node started (idle).")

    def set_active(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / self.rate, self.update)
        self.state_pub.publish(Bool(data=True))

    def set_idle(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        if self.tf_listener is not None:
            # TransformListener has no public destroy; tear down subscriptions directly
            for sub_attr in ('tf_sub', 'tf_static_sub'):
                sub = getattr(self.tf_listener, sub_attr, None)
                if sub is not None:
                    self.destroy_subscription(sub)
            self.tf_listener = None
            self.tf_buffer = None

        self.waypoints = []
        self.current_goal = None
        self.state_pub.publish(Bool(data=False))

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

        # If already active, reset cleanly before starting new path
        if self.timer is not None:
            self.set_idle()

        self.waypoints = poses
        self.waypoints_header = msg.header
        self.set_active()
        self.get_logger().info("New path received.")

    def estop_callback(self, _):
        if self.timer is None:
            return

        if self.current_goal and self.last_robot_pose and self.waypoints_header:
            self.send_goal(self.last_robot_pose)
            
        self.set_idle()
        self.get_logger().info("Emergency stop triggered.")

    def goal_pose_callback(self, msg):
        if self.current_goal is None:
            return
        if not pose_equals(msg, self.current_goal):
            self.get_logger().warn("External goal preemption detection, aborting path.")
            self.set_idle()

    def send_goal(self, pose):
        goal = PoseStamped()
        goal.header = self.waypoints_header
        goal.pose = pose
        self.goal_pub.publish(goal)
        self.current_goal = goal

    def update(self):
        if self.current_goal is None and self.waypoints:
            next_pose = self.waypoints[0]
            self.send_goal(next_pose)
            self.get_logger().info(f"Published: ({next_pose.position.x},{next_pose.position.y},{next_pose.position.z}), {len(self.waypoints)} goals left.")

        if self.current_goal and self.check_goal_reached(self.current_goal):
            self.waypoints.pop(0)
            self.current_goal = None

            if not self.waypoints:
                self.get_logger().info("Path finished")
                self.set_idle()

    def check_goal_reached(self, goal):
        try:
            transform = self.tf_buffer.lookup_transform(goal.header.frame_id, self.robot_link, rclpy.time.Time())

            pos = transform.transform.translation
            self.last_robot_pose = Pose()
            self.last_robot_pose.position.x = pos.x
            self.last_robot_pose.position.y = pos.y
            self.last_robot_pose.position.z = pos.z
            self.last_robot_pose.orientation = transform.transform.rotation

            dx = pos.x - goal.pose.position.x
            dy = pos.y - goal.pose.position.y
            return math.sqrt(dx * dx + dy * dy) <= self.goal_reached_range
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsToSimpleGoals()
    rclpy.spin(node)
    node.set_idle()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()