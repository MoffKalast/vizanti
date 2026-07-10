#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )

class TriangleListTestPublisher(Node):
    def __init__(self):
        super().__init__("triangle_list_test_publisher")

        latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
            MarkerArray,
            "/test_triangle_list",
            latch,
        )

        self.angle = 0.0
        self.angular_speed = 0.8  # rad/s
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(1/30.0, self.timer_callback)

    def timer_callback(self):
        now_time = self.get_clock().now()
        dt = (now_time - self.last_time).nanoseconds * 1e-9
        self.last_time = now_time

        self.angle += self.angular_speed * dt
        #self.angle = self.angle % (2.0 * math.pi)

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ------------------------------------------------------------
        # Test 1: Flat quad (2 triangles, single color)
        # ------------------------------------------------------------
        marker1 = Marker()
        marker1.header.frame_id = "map"
        marker1.header.stamp = now
        marker1.ns = "flat_quad"
        marker1.id = 1
        marker1.type = Marker.TRIANGLE_LIST
        marker1.action = Marker.ADD
        marker1.pose.orientation.w = 1.0
        marker1.scale.x = 1.0
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        p0 = Point(x=0.0, y=0.0, z=0.0)
        p1 = Point(x=1.0, y=0.0, z=0.0)
        p2 = Point(x=1.0, y=1.0, z=0.0)
        p3 = Point(x=0.0, y=1.0, z=0.0)

        marker1.points.extend([p0, p1, p2])
        marker1.points.extend([p0, p2, p3])

        marker_array.markers.append(marker1)

        # ------------------------------------------------------------
        # Test 2: Per-vertex colored triangle fan
        # ------------------------------------------------------------
        marker2 = Marker()
        marker2.header.frame_id = "map"
        marker2.header.stamp = now
        marker2.ns = "fan thing"
        marker2.id = 2
        marker2.type = Marker.TRIANGLE_LIST
        marker2.action = Marker.ADD
        marker2.scale.x = 1.0
        marker2.scale.y = 1.5
        marker2.scale.z = 1.0

        qx, qy, qz, qw = euler_to_quaternion(self.angle, self.angle, self.angle)
        marker2.pose.orientation.x = qx
        marker2.pose.orientation.y = qy
        marker2.pose.orientation.z = qz
        marker2.pose.orientation.w = qw
        marker2.pose.position.x = -3.0

        center = Point(x=0.0, y=0.0, z=1.0)
        num_triangles = 12
        radius = 0.8

        for i in range(num_triangles):
            a0 = i * 2.0 * math.pi / num_triangles
            a1 = (i + 1) * 2.0 * math.pi / num_triangles

            p0 = center
            p1 = Point(
                x=center.x + math.cos(a0) * radius,
                y=center.y + math.sin(a0) * radius,
                z=0.0,
            )
            p2 = Point(
                x=center.x + math.cos(a1) * radius,
                y=center.y + math.sin(a1) * radius,
                z=0.0,
            )

            marker2.points.extend([p0, p1, p2])

            # MUST match points length exactly
            for j in range(3):
                hue = (i + j / 3.0) / num_triangles
                marker2.colors.append(
                    ColorRGBA(
                        r=abs(math.sin(hue * math.pi)),
                        g=abs(math.sin(hue * math.pi + 2.0)),
                        b=abs(math.sin(hue * math.pi + 4.0)),
                        a=1.0,
                    )
                )

        marker_array.markers.append(marker2)

        # ------------------------------------------------------------
        # Test 3: 3D pyramid (closed mesh)
        # ------------------------------------------------------------
        marker3 = Marker()
        marker3.header.frame_id = "map"
        marker3.header.stamp = now
        marker3.ns = "pyramid"
        marker3.id = 3
        marker3.type = Marker.TRIANGLE_LIST
        marker3.action = Marker.ADD
        marker3.scale.x = 1.0
        marker3.scale.y = 1.0
        marker3.scale.z = 1.0
        marker3.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        qx, qy, qz, qw = euler_to_quaternion(self.angle, 0.0, 0.0)
        marker3.pose.orientation.x = qx
        marker3.pose.orientation.y = qy
        marker3.pose.orientation.z = qz
        marker3.pose.orientation.w = qw


        top = Point(x=4.0, y=0.0, z=1.0)
        base = [
            Point(x=3.5, y=-0.5, z=0.0),
            Point(x=4.5, y=-0.5, z=0.0),
            Point(x=4.5, y=0.5, z=0.0),
            Point(x=3.5, y=0.5, z=0.0),
        ]

        # Base
        marker3.points.extend([base[0], base[1], base[2]])
        marker3.points.extend([base[0], base[2], base[3]])

        # Sides
        for i in range(4):
            p0 = base[i]
            p1 = base[(i + 1) % 4]
            marker3.points.extend([p0, p1, top])

        marker_array.markers.append(marker3)

        # ------------------------------------------------------------
        # Test 4: Degenerate triangle (valid but zero area)
        # ------------------------------------------------------------
        marker4 = Marker()
        marker4.header.frame_id = "map"
        marker4.header.stamp = now
        marker4.ns = ""
        marker4.id = 4
        marker4.type = Marker.TRIANGLE_LIST
        marker4.action = Marker.ADD
        marker4.pose.orientation.w = 1.0
        marker4.scale.x = 1.0
        marker4.scale.y = 1.0
        marker4.scale.z = 1.0
        marker4.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        p = Point(x=6.0, y=0.0, z=0.0)
        marker4.points.extend([p, p, p])

        marker_array.markers.append(marker4)

        self.pub.publish(marker_array)
        self.get_logger().info(
            f"Published {len(marker_array.markers)} triangle test markers"
        )


def main():
    rclpy.init()
    node = TriangleListTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()