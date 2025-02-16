#!/usr/bin/env python3

import math
import rospy
import tf2_ros

from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from std_msgs.msg import Bool, Empty

"""
Since many navigations stacks do not support multiple goals or even action servers,
this node serves is a demo example of sending simple goals sequentially, checking for range completion in between.
Goal timeouts and handling other edge cases is left as an excercise to the reader.
"""

class WaypointsToSimpleGoals:
	def __init__(self):
		rospy.init_node('waypoints_to_simple_goals')
		
		self.rate = rospy.get_param('~rate', 10)
		self.robot_link = rospy.get_param('~robot_link', 'base_link')
		self.goal_reached_range = rospy.get_param('~goal_reached_range', 0.1)
		
		# State management through waypoints list
		self.waypoints = []
		self.current_goal = None
		self.waypoints_header = None

		self.last_robot_pose = None
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		
		# Remap this if your nav stack takes simple goals on another topic
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

		# Sends out the current navigation state (active/idle)
		self.state_pub = rospy.Publisher('/waypoints/state', Bool, queue_size=1, latch=True)
		
		# Point the Waypoints widget to this one
		self.waypoints_sub = rospy.Subscriber('/waypoints', PoseArray, self.waypoints_callback)

		# Point a Button widget to this one to stop path execution at any time
		self.estop_sub = rospy.Subscriber('/waypoints/estop', Empty, self.estop_callback)
		
		# Initialize state
		self.publish_navigation_state(False)
		
		rospy.loginfo("Waypoints node started.")

	def waypoints_callback(self, msg):
		poses = list(msg.poses)

		if len(poses) == 0:
			#Empty array was sent as a preempt, we should just abort current execution
			self.estop_callback(None)
			return

		self.waypoints = poses
		self.waypoints_header = msg.header
		self.current_goal = None
		rospy.loginfo("New path received!")

	def estop_callback(self, _):
		if self.current_goal is not None and self.last_robot_pose is not None and self.waypoints_header is not None:
			goal = PoseStamped()
			goal.header = self.waypoints_header
			goal.pose = self.last_robot_pose
			self.goal_pub.publish(goal)

		self.waypoints = []
		self.current_goal = None
		self.publish_navigation_state(False)

		rospy.loginfo("Emergency stop triggered!")

	def update(self):
		if not self.waypoints:
			if self.current_goal is not None:
				self.current_goal = None
				self.publish_navigation_state(False)
			return

		# Send next goal
		if self.current_goal is None and self.waypoints:
			
			next_pose = self.waypoints[0]
			
			goal = PoseStamped()
			goal.header = self.waypoints_header
			goal.pose = next_pose
			self.goal_pub.publish(goal)
			
			self.current_goal = goal
			self.publish_navigation_state(True)
			rospy.loginfo(f"Published: ({next_pose.position.x},{next_pose.position.y},{next_pose.position.z}), {len(self.waypoints)} goals left.")

		# Check if current goal is reached
		if self.current_goal and self.check_goal_reached(self.current_goal):
			self.waypoints.pop(0)
			self.current_goal = None

			if not self.waypoints:
				rospy.loginfo("Path finished!")

	def check_goal_reached(self, goal):
		try:
			transform = self.tf_buffer.lookup_transform(
				goal.header.frame_id,
				self.robot_link,
				rospy.Time(0),  # get most recent transform
				rospy.Duration(1.0)
			)

			self.last_robot_pose = Pose()
			self.last_robot_pose.position = transform.transform.translation
			self.last_robot_pose.orientation = transform.transform.rotation

			dx = transform.transform.translation.x - goal.pose.position.x
			dy = transform.transform.translation.y - goal.pose.position.y
			distance = math.sqrt(dx*dx + dy*dy)

			return distance <= self.goal_reached_range

		except (tf2_ros.LookupException, 
				tf2_ros.ConnectivityException,
				tf2_ros.ExtrapolationException) as e:
			return False

	def publish_navigation_state(self, state):
		self.state_pub.publish(Bool(data=state))

	def cleanup(self):
		self.waypoints = []
		self.current_goal = None
		self.publish_navigation_state(False)

node = WaypointsToSimpleGoals()
rate = rospy.Rate(node.rate)
rospy.on_shutdown(node.cleanup)

while not rospy.is_shutdown():
	node.update()
	rate.sleep()