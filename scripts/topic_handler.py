#!/usr/bin/env python3

import rospy
import time
import threading

from tf2_msgs.msg import TFMessage

class TfConsolidator:

	def __init__(self):
		rospy.init_node('vizanti_tf_handler_node')

		self.updated = False
		self.lock = threading.Lock()
		self.transforms = {}

		self.transform_timeout = {}
		self.timeout_prescaler = 0

		self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
		self.tf_pub = rospy.Publisher('/vizanti/tf_consolidated', TFMessage, queue_size=1, latch=True)

		rospy.loginfo("TF handler ready.")

	def clear_old_tfs(self):
		with self.lock:
			for key in list(self.transforms.keys()):
				if time.monotonic() - self.transform_timeout[key] > 10.0:
					parent = self.transforms[key].header.frame_id
					del self.transforms[key]
					del self.transform_timeout[key]
					self.updated = True
					rospy.logwarn("Deleted old TF link: "+str(parent)+" -> "+str(key))

	def publish(self):
		# 150 /30 hz = once per 5 seconds
		self.timeout_prescaler = (self.timeout_prescaler + 1) % 150
		if self.timeout_prescaler == 0:
			self.clear_old_tfs()

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
				self.transform_timeout[transform.child_frame_id] = time.monotonic()
			self.updated = True
		

odr = TfConsolidator()
period = 1.0 / 30.0
deadline = time.monotonic()

while not rospy.is_shutdown():
	try:
		odr.publish()
	except Exception as e:
		rospy.logerr("Failed to publish consolidated TF: "+str(e))

	deadline += period
	sleep_time = deadline - time.monotonic()

	if sleep_time > 0:
		time.sleep(sleep_time)
	else:
		deadline = time.monotonic()