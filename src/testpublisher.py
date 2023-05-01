#!/usr/bin/env python3


import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class TestPub:

	def __init__(self):
		rospy.init_node('testpub', anonymous=True)

		self.publisher = rospy.Publisher('/line_marker_array', MarkerArray, queue_size=10)

	def publish(self):
		line_marker = Marker()
		line_marker.header.frame_id = "base_link"
		line_marker.header.stamp = rospy.Time.now()
		line_marker.ns = "line_marker"
		line_marker.id = 0
		line_marker.type = Marker.LINE_STRIP
		line_marker.action = Marker.ADD

		line_marker.points = [
			Point(0, 0, 0),
			Point(1, 1, 0),
			Point(2, 0, 0)
		]

		line_marker.scale.x = 0.05  # Line width

		line_marker.colors = [
			ColorRGBA(0.3,0.3,0.3,1),
			ColorRGBA(0.3,0.3,0.3,1),
			ColorRGBA(0.3,0.3,0.3,1)
		]

		line_marker.lifetime = rospy.Duration(0)

		marker_array = MarkerArray()
		marker_array.markers.append(line_marker)

		rate = rospy.Rate(1)  # 1 Hz

		self.publisher.publish(marker_array)
		

odr = TestPub()
rate = rospy.Rate(1)
while not rospy.is_shutdown():
	odr.publish()
	rate.sleep()
	