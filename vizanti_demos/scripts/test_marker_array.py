#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import random

class MarkerArrayPublisher(Node):
    def __init__(self):
        super().__init__('test_marker_array')

        latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.marker_array_publisher = self.create_publisher(MarkerArray, 'test_marker_array', latch)

        marker_array = MarkerArray()

        # Define particle types and their variations
        particle_types = [Marker.ARROW, Marker.CUBE, Marker.CYLINDER, Marker.SPHERE, Marker.TEXT_VIEW_FACING]
        rotations = [(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.707, 0.707), (0.0, 0.0, 0.383, 0.924)]
        num_variations = 5

        for i, particle_type in enumerate(particle_types):
            for j in range(num_variations):
                marker = Marker()
                marker.header.frame_id = 'test_link'
                marker.id = i * num_variations + j
                marker.type = particle_type
                marker.action = Marker.ADD
                marker.pose.position.x = i * 2.0  # Separation on the X axis
                marker.pose.position.y = j * 2.0  # Separation on the Y axis
                marker.pose.position.z = 0.0
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = rotations[random.randint(0,len(rotations)-1)]
                marker.scale.x, marker.scale.y, marker.scale.z = self.randomScale()
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.randomColour()
                marker.lifetime.sec = 6000
                marker.text = "TEST"
                marker_array.markers.append(marker)

        self.marker_array_publisher.publish(marker_array)

    def randomScale(self):
        return (random.uniform(0.2,1.5), random.uniform(0.2,1.5), random.uniform(0.2,1.5))

    def randomColour(self):
        return (random.random(), random.random(), random.random(), random.uniform(0.3,1.0))


def main(args=None):
    rclpy.init(args=args)
    marker_array_publisher = MarkerArrayPublisher()
    rclpy.spin(marker_array_publisher)

if __name__ == "__main__":
    main()