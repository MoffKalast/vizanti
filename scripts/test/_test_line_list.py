 #!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def create_line_list_marker(marker_id, points, colors=None, color=None, scale_x=0.05, ns="line_list_test"):
    marker = Marker()
    marker.header.frame_id = "local"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = scale_x  # Line width
    
    if color:
        marker.color = color
    
    marker.points = points
    if colors:
        marker.colors = colors
    
    return marker

def main():
    rospy.init_node('line_list_test_publisher')
    pub = rospy.Publisher('/test_line_markers', MarkerArray, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        
        # Test 1: Simple LINE_LIST with solid color
        points1 = [
            Point(0, 0, 0), Point(1, 0, 0),  # Line 1: horizontal
            Point(0, 0.5, 0), Point(1, 0.5, 0),  # Line 2: horizontal
            Point(0, 1, 0), Point(1, 1, 0),  # Line 3: horizontal
        ]
        marker1 = create_line_list_marker(
            marker_id=1,
            points=points1,
            color=ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red
            scale_x=0.05
        )
        marker_array.markers.append(marker1)
        
        # Test 2: LINE_LIST with per-vertex colors (gradient test)
        points2 = [
            Point(2, 0, 0), Point(3, 0, 0),  # Line 1
            Point(2, 0.5, 0), Point(3, 0.5, 0),  # Line 2
            Point(2, 1, 0), Point(3, 1, 0),  # Line 3
        ]
        colors2 = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0), ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Red to Green
            ColorRGBA(0.0, 0.0, 1.0, 1.0), ColorRGBA(1.0, 1.0, 0.0, 1.0),  # Blue to Yellow
            ColorRGBA(1.0, 0.0, 1.0, 1.0), ColorRGBA(0.0, 1.0, 1.0, 1.0),  # Magenta to Cyan
        ]
        marker2 = create_line_list_marker(
            marker_id=2,
            points=points2,
            colors=colors2,
            scale_x=0.05
        )
        marker_array.markers.append(marker2)
        
        # Test 3: LINE_LIST with different line widths
        points3 = [
            Point(4, 0, 0), Point(5, 0, 0),
            Point(4, 0.5, 0), Point(5, 0.5, 0),
            Point(4, 1, 0), Point(5, 1, 0),
        ]
        marker3 = create_line_list_marker(
            marker_id=3,
            points=points3,
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
            scale_x=0.15  # Thicker lines
        )
        marker_array.markers.append(marker3)
        
        # Test 4: LINE_LIST in 3D (with Z variation)
        points4 = [
            Point(0, 2, 0), Point(1, 2, 1),
            Point(0, 2.5, 0.5), Point(1, 2.5, 1.5),
            Point(0, 3, 1), Point(1, 3, 0),
        ]
        colors4 = [
            ColorRGBA(1.0, 1.0, 1.0, 1.0), ColorRGBA(0.0, 0.0, 0.0, 1.0),  # White to Black
            ColorRGBA(1.0, 0.5, 0.0, 1.0), ColorRGBA(0.5, 0.0, 1.0, 1.0),  # Orange to Purple
            ColorRGBA(0.0, 1.0, 0.5, 1.0), ColorRGBA(1.0, 0.0, 0.5, 1.0),  # Teal to Rose
        ]
        marker4 = create_line_list_marker(
            marker_id=4,
            points=points4,
            colors=colors4,
            scale_x=0.08
        )
        marker_array.markers.append(marker4)
        
        # Test 5: Comparison - LINE_STRIP vs LINE_LIST side by side
        # LINE_LIST version
        points5a = [
            Point(6, 0, 0), Point(6.3, 0.3, 0),
            Point(6.3, 0.3, 0), Point(6.6, 0, 0),
            Point(6.6, 0, 0), Point(7, 0.5, 0),
        ]
        marker5a = create_line_list_marker(
            marker_id=5,
            points=points5a,
            color=ColorRGBA(1.0, 0.0, 1.0, 1.0),  # Magenta
            scale_x=0.05,
            ns="line_list_comparison"
        )
        marker_array.markers.append(marker5a)
        
        # LINE_STRIP version (for comparison)
        marker5b = Marker()
        marker5b.header.frame_id = "local"
        marker5b.header.stamp = rospy.Time.now()
        marker5b.ns = "line_strip_comparison"
        marker5b.id = 6
        marker5b.type = Marker.LINE_STRIP
        marker5b.action = Marker.ADD
        marker5b.pose.orientation.w = 1.0
        marker5b.scale.x = 0.05
        marker5b.color = ColorRGBA(0.0, 1.0, 1.0, 1.0)  # Cyan
        marker5b.points = [
            Point(6, 0.8, 0), Point(6.3, 1.1, 0),
            Point(6.6, 0.8, 0), Point(7, 1.3, 0),
        ]
        marker_array.markers.append(marker5b)
        
        pub.publish(marker_array)
        rospy.loginfo("Published %d test markers", len(marker_array.markers))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass