#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import random

def random_scale():
    return (random.uniform(0.2, 1.5), random.uniform(0.2, 1.5), random.uniform(0.2, 1.5))

def random_colour():
    return (random.random(), random.random(), random.random(), random.uniform(0.3, 1.0))

def publish_marker_array():
    rospy.init_node('test_marker_array', anonymous=True)

    pub = rospy.Publisher('test_marker_array', MarkerArray, queue_size=1, latch=True)
    rospy.sleep(1.0)  # Allow publisher to register with master

    marker_array = MarkerArray()

    particle_types = [Marker.ARROW, Marker.CUBE, Marker.CYLINDER, Marker.SPHERE, Marker.TEXT_VIEW_FACING]
    rotations = [
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.707, 0.707),
        (0.0, 0.0, 0.383, 0.924)
    ]
    num_variations = 5

    for i, particle_type in enumerate(particle_types):
        for j in range(num_variations):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.id = i * num_variations + j
            marker.type = particle_type
            marker.action = Marker.ADD
            marker.pose.position.x = i * 2.0
            marker.pose.position.y = j * 2.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = random.choice(rotations)
            marker.scale.x, marker.scale.y, marker.scale.z = random_scale()
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = random_colour()
            marker.lifetime = rospy.Duration(6000)
            if particle_type == Marker.TEXT_VIEW_FACING:
                marker.text = "TEST"
            marker_array.markers.append(marker)

    pub.publish(marker_array)
    rospy.loginfo("Published MarkerArray with %d markers.", len(marker_array.markers))

if __name__ == '__main__':
    try:
        publish_marker_array()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
