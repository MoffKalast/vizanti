#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class SphereListTestPublisher(Node):
    def __init__(self):
        super().__init__("sphere_list_test_publisher")

        latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
            MarkerArray,
            "/test_sphere_list",
            latch,
        )

        # 1 Hz timer
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Test 1: SPHERE_LIST with uniform color
        marker1 = Marker()
        marker1.header.frame_id = "world"
        marker1.header.stamp = now
        marker1.ns = "sphere_list_test"
        marker1.id = 1
        marker1.type = Marker.SPHERE_LIST
        marker1.action = Marker.ADD
        marker1.pose.orientation.w = 1.0
        marker1.scale.x = 0.2
        marker1.scale.y = 0.2
        marker1.scale.z = 0.2
        marker1.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        for i in range(5):
            for j in range(5):
                marker1.points.append(Point(x=i * 0.3, y=j * 0.3, z=0.0))

        marker_array.markers.append(marker1)

        # Test 2: SPHERE_LIST with per-sphere colors
        marker2 = Marker()
        marker2.header.frame_id = "world"
        marker2.header.stamp = now
        marker2.ns = "sphere_list_test"
        marker2.id = 2
        marker2.type = Marker.SPHERE_LIST
        marker2.action = Marker.ADD
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = 0.15
        marker2.scale.y = 0.15
        marker2.scale.z = 0.15

        for i in range(10):
            x = 2.0 + i * 0.2
            y = 0.5
            marker2.points.append(Point(x=x, y=y, z=0.0))

            hue = i / 10.0
            if hue < 1.0 / 6.0:
                color = ColorRGBA(r=1.0, g=hue * 6.0, b=0.0, a=1.0)
            elif hue < 2.0 / 6.0:
                color = ColorRGBA(r=(2.0 / 6.0 - hue) * 6.0, g=1.0, b=0.0, a=1.0)
            elif hue < 3.0 / 6.0:
                color = ColorRGBA(r=0.0, g=1.0, b=(hue - 2.0 / 6.0) * 6.0, a=1.0)
            elif hue < 4.0 / 6.0:
                color = ColorRGBA(r=0.0, g=(4.0 / 6.0 - hue) * 6.0, b=1.0, a=1.0)
            elif hue < 5.0 / 6.0:
                color = ColorRGBA(r=(hue - 4.0 / 6.0) * 6.0, g=0.0, b=1.0, a=1.0)
            else:
                color = ColorRGBA(r=1.0, g=0.0, b=(1.0 - hue) * 6.0, a=1.0)

            marker2.colors.append(color)

        marker_array.markers.append(marker2)

        # Test 3: SPHERE_LIST with larger scale
        marker3 = Marker()
        marker3.header.frame_id = "world"
        marker3.header.stamp = now
        marker3.ns = "sphere_list_test"
        marker3.id = 3
        marker3.type = Marker.SPHERE_LIST
        marker3.action = Marker.ADD
        marker3.pose.orientation.w = 1.0
        marker3.scale.x = 0.3
        marker3.scale.y = 0.3
        marker3.scale.z = 0.3
        marker3.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)

        for i in range(12):
            angle = i * 2.0 * math.pi / 12.0
            x = 2.0 + math.cos(angle) * 0.8
            y = 2.0 + math.sin(angle) * 0.8
            marker3.points.append(Point(x=x, y=y, z=0.0))

        marker_array.markers.append(marker3)

        # Test 4: SPHERE_LIST in 3D with varying Z
        marker4 = Marker()
        marker4.header.frame_id = "world"
        marker4.header.stamp = now
        marker4.ns = "sphere_list_test"
        marker4.id = 4
        marker4.type = Marker.SPHERE_LIST
        marker4.action = Marker.ADD
        marker4.pose.orientation.w = 1.0
        marker4.scale.x = 0.1
        marker4.scale.y = 0.1
        marker4.scale.z = 0.1

        for i in range(20):
            angle = i * 0.5
            radius = 0.5 + i * 0.05
            x = 4.0 + math.cos(angle) * radius
            y = 2.0 + math.sin(angle) * radius
            z = i * 0.1
            marker4.points.append(Point(x=x, y=y, z=z))

            intensity = i / 20.0
            marker4.colors.append(
                ColorRGBA(r=intensity, g=0.5, b=1.0 - intensity, a=1.0)
            )

        marker_array.markers.append(marker4)

        # Test 5: Individual SPHERE markers (comparison)
        for i in range(3):
            marker_single = Marker()
            marker_single.header.frame_id = "world"
            marker_single.header.stamp = now
            marker_single.ns = "single_spheres"
            marker_single.id = 10 + i
            marker_single.type = Marker.SPHERE
            marker_single.action = Marker.ADD
            marker_single.pose.position = Point(x=i * 0.5, y=4.0, z=0.0)
            marker_single.pose.orientation.w = 1.0
            marker_single.scale.x = 0.2
            marker_single.scale.y = 0.2
            marker_single.scale.z = 0.2
            marker_single.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

            marker_array.markers.append(marker_single)

        self.pub.publish(marker_array)
        self.get_logger().info(f"Published {len(marker_array.markers)} test markers")


def main():
    rclpy.init()
    node = SphereListTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()