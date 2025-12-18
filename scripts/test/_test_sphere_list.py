#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

def main():
    rospy.init_node('sphere_list_test_publisher')
    pub = rospy.Publisher('/test_sphere_list', MarkerArray, queue_size=10)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        
        # Test 1: SPHERE_LIST with uniform color
        marker1 = Marker()
        marker1.header.frame_id = "local"
        marker1.header.stamp = rospy.Time.now()
        marker1.ns = "sphere_list_test"
        marker1.id = 1
        marker1.type = Marker.SPHERE_LIST
        marker1.action = Marker.ADD
        marker1.pose.orientation.w = 1.0
        marker1.scale.x = 0.2  # Diameter
        marker1.scale.y = 0.2
        marker1.scale.z = 0.2
        marker1.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        
        # Create a grid of spheres
        for i in range(5):
            for j in range(5):
                marker1.points.append(Point(i * 0.3, j * 0.3, 0))
        
        marker_array.markers.append(marker1)
        
        # Test 2: SPHERE_LIST with per-sphere colors
        marker2 = Marker()
        marker2.header.frame_id = "local"
        marker2.header.stamp = rospy.Time.now()
        marker2.ns = "sphere_list_test"
        marker2.id = 2
        marker2.type = Marker.SPHERE_LIST
        marker2.action = Marker.ADD
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = 0.15
        marker2.scale.y = 0.15
        marker2.scale.z = 0.15
        
        # Rainbow colored spheres in a line
        for i in range(10):
            x = 2.0 + i * 0.2
            y = 0.5
            marker2.points.append(Point(x, y, 0))
            
            # Create rainbow colors
            hue = i / 10.0
            if hue < 1.0/6:
                color = ColorRGBA(1.0, hue*6, 0.0, 1.0)
            elif hue < 2.0/6:
                color = ColorRGBA((2.0/6 - hue)*6, 1.0, 0.0, 1.0)
            elif hue < 3.0/6:
                color = ColorRGBA(0.0, 1.0, (hue - 2.0/6)*6, 1.0)
            elif hue < 4.0/6:
                color = ColorRGBA(0.0, (4.0/6 - hue)*6, 1.0, 1.0)
            elif hue < 5.0/6:
                color = ColorRGBA((hue - 4.0/6)*6, 0.0, 1.0, 1.0)
            else:
                color = ColorRGBA(1.0, 0.0, (1.0 - hue)*6, 1.0)
            
            marker2.colors.append(color)
        
        marker_array.markers.append(marker2)
        
        # Test 3: SPHERE_LIST with different scales
        marker3 = Marker()
        marker3.header.frame_id = "local"
        marker3.header.stamp = rospy.Time.now()
        marker3.ns = "sphere_list_test"
        marker3.id = 3
        marker3.type = Marker.SPHERE_LIST
        marker3.action = Marker.ADD
        marker3.pose.orientation.w = 1.0
        marker3.scale.x = 0.3  # Larger spheres
        marker3.scale.y = 0.3
        marker3.scale.z = 0.3
        marker3.color = ColorRGBA(0.0, 1.0, 0.0, 0.7)  # Semi-transparent green
        
        # Circle pattern
        for i in range(12):
            angle = i * 2 * math.pi / 12
            x = 2.0 + math.cos(angle) * 0.8
            y = 2.0 + math.sin(angle) * 0.8
            marker3.points.append(Point(x, y, 0))
        
        marker_array.markers.append(marker3)
        
        # Test 4: SPHERE_LIST in 3D (with varying Z)
        marker4 = Marker()
        marker4.header.frame_id = "local"
        marker4.header.stamp = rospy.Time.now()
        marker4.ns = "sphere_list_test"
        marker4.id = 4
        marker4.type = Marker.SPHERE_LIST
        marker4.action = Marker.ADD
        marker4.pose.orientation.w = 1.0
        marker4.scale.x = 0.1
        marker4.scale.y = 0.1
        marker4.scale.z = 0.1
        
        # Spiral of spheres with height variation
        for i in range(20):
            angle = i * 0.5
            radius = 0.5 + i * 0.05
            x = 4.0 + math.cos(angle) * radius
            y = 2.0 + math.sin(angle) * radius
            z = i * 0.1
            marker4.points.append(Point(x, y, z))
            
            # Color based on height
            intensity = i / 20.0
            marker4.colors.append(ColorRGBA(intensity, 0.5, 1.0 - intensity, 1.0))
        
        marker_array.markers.append(marker4)
        
        # Test 5: Comparison - individual SPHERE markers vs SPHERE_LIST
        # Individual spheres (slower)
        for i in range(3):
            marker_single = Marker()
            marker_single.header.frame_id = "local"
            marker_single.header.stamp = rospy.Time.now()
            marker_single.ns = "single_spheres"
            marker_single.id = 10 + i
            marker_single.type = Marker.SPHERE
            marker_single.action = Marker.ADD
            marker_single.pose.position = Point(i * 0.5, 4.0, 0)
            marker_single.pose.orientation.w = 1.0
            marker_single.scale.x = 0.2
            marker_single.scale.y = 0.2
            marker_single.scale.z = 0.2
            marker_single.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow
            marker_array.markers.append(marker_single)
        
        pub.publish(marker_array)
        rospy.loginfo("Published %d test markers", len(marker_array.markers))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass