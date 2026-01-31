#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def create_line_list_marker(
    marker_id,
    points,
    colors=None,
    color=None,
    scale_x=0.05,
    ns="line_list_test",
    frame_id="world",
    stamp=None,
):
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp

    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = scale_x  # Line width

    if color is not None:
        marker.color = color

    marker.points = points
    if colors is not None:
        marker.colors = colors

    return marker


class LineListTestPublisher(Node):
    def __init__(self):
        super().__init__("line_list_test_publisher")

        latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
            MarkerArray,
            "/test_line_markers",
            latch,
        )

        # 1 Hz timer
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Test 1: Simple LINE_LIST with solid color
        points1 = [
            Point(x=0.0, y=0.0, z=0.0), Point(x=1.0, y=0.0, z=0.0),
            Point(x=0.0, y=0.5, z=0.0), Point(x=1.0, y=0.5, z=0.0),
            Point(x=0.0, y=1.0, z=0.0), Point(x=1.0, y=1.0, z=0.0),
        ]
        marker1 = create_line_list_marker(
            marker_id=1,
            points=points1,
            color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            scale_x=0.05,
            stamp=now,
        )
        marker_array.markers.append(marker1)

        # Test 2: LINE_LIST with per-vertex colors
        points2 = [
            Point(x=2.0, y=0.0, z=0.0), Point(x=3.0, y=0.0, z=0.0),
            Point(x=2.0, y=0.5, z=0.0), Point(x=3.0, y=0.5, z=0.0),
            Point(x=2.0, y=1.0, z=0.0), Point(x=3.0, y=1.0, z=0.0),
        ]
        colors2 = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0), ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
        ]
        marker2 = create_line_list_marker(
            marker_id=2,
            points=points2,
            colors=colors2,
            scale_x=0.05,
            stamp=now,
        )
        marker_array.markers.append(marker2)

        # Test 3: Different line widths
        points3 = [
            Point(x=4.0, y=0.0, z=0.0), Point(x=5.0, y=0.0, z=0.0),
            Point(x=4.0, y=0.5, z=0.0), Point(x=5.0, y=0.5, z=0.0),
            Point(x=4.0, y=1.0, z=0.0), Point(x=5.0, y=1.0, z=0.0),
        ]
        marker3 = create_line_list_marker(
            marker_id=3,
            points=points3,
            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            scale_x=0.15,
            stamp=now,
        )
        marker_array.markers.append(marker3)

        # Test 4: 3D LINE_LIST
        points4 = [
            Point(x=0.0, y=2.0, z=0.0), Point(x=1.0, y=2.0, z=1.0),
            Point(x=0.0, y=2.5, z=0.5), Point(x=1.0, y=2.5, z=1.5),
            Point(x=0.0, y=3.0, z=1.0), Point(x=1.0, y=3.0, z=0.0),
        ]
        colors4 = [
            ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0), ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0), ColorRGBA(r=0.5, g=0.0, b=1.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0), ColorRGBA(r=1.0, g=0.0, b=0.5, a=1.0),
        ]
        marker4 = create_line_list_marker(
            marker_id=4,
            points=points4,
            colors=colors4,
            scale_x=0.08,
            stamp=now,
        )
        marker_array.markers.append(marker4)

        # Test 5: LINE_LIST vs LINE_STRIP comparison
        points5a = [
            Point(x=6.0, y=0.0, z=0.0), Point(x=6.3, y=0.3, z=0.0),
            Point(x=6.3, y=0.3, z=0.0), Point(x=6.6, y=0.0, z=0.0),
            Point(x=6.6, y=0.0, z=0.0), Point(x=7.0, y=0.5, z=0.0),
        ]
        marker5a = create_line_list_marker(
            marker_id=5,
            points=points5a,
            color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
            scale_x=0.05,
            ns="line_list_comparison",
            stamp=now,
        )
        marker_array.markers.append(marker5a)

        marker5b = Marker()
        marker5b.header.frame_id = "world"
        marker5b.header.stamp = now
        marker5b.ns = "line_strip_comparison"
        marker5b.id = 6
        marker5b.type = Marker.LINE_STRIP
        marker5b.action = Marker.ADD
        marker5b.pose.orientation.w = 1.0
        marker5b.scale.x = 0.05
        marker5b.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        marker5b.points = [
            Point(x=6.0, y=0.8, z=0.0),
            Point(x=6.3, y=1.1, z=0.0),
            Point(x=6.6, y=0.8, z=0.0),
            Point(x=7.0, y=1.3, z=0.0),
        ]
        marker_array.markers.append(marker5b)

        self.pub.publish(marker_array)
        self.get_logger().info(
            f"Published {len(marker_array.markers)} test markers"
        )


def main():
    rclpy.init()
    node = LineListTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()