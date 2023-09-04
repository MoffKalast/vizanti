#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
import threading

class TopicHandler:

	def __init__(self):
		rospy.init_node('vizanti_topic_handler')

		self.updated = False
		self.lock = threading.Lock()
		self.transforms = {}

		self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
		self.tf_pub = rospy.Publisher('/vizanti/tf_consolidated', TFMessage, queue_size=1)

		rospy.loginfo("TF handler ready.")

	def publish(self):
		if not self.updated:
			return

		msg = TFMessage()
		with self.lock:
			msg.transforms = list(self.transforms.values())
			self.updated = False

		self.tf_pub.publish(msg)

	def tf_callback(self, msg):
		with self.lock:
			for transform in msg.transforms:
				self.transforms[transform.child_frame_id] = transform
			self.updated = True
		

odr = TopicHandler()
rate = rospy.Rate(30)
while not rospy.is_shutdown():
	odr.publish();
	rate.sleep()