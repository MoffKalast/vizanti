#!/usr/bin/env python3

import os
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from ament_index_python.packages import get_package_share_directory

class ImageToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('test_occupancy_grid')

        # Declare and get parameters
        self.declare_parameter('image_path', 'assets/test_heightmap.png')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('topic_name', '/heightmap')
        self.declare_parameter('scale', 0.1)

        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        scale = self.get_parameter('scale').get_parameter_value().double_value

        self.get_logger().info("Loading image...")

        absolute_path = os.path.join(get_package_share_directory('vizanti_demos'), image_path)

        print(absolute_path)
        image = cv2.imread(absolute_path, cv2.IMREAD_GRAYSCALE)

        # Flatten the image and convert to occupancy grid data
        data = []
        for pixel in image.flatten():
            data.append(pixel)

        # Create OccupancyGrid message
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = frame_id
        grid.info.resolution = float(scale)
        grid.info.width = image.shape[1]
        grid.info.height = image.shape[0]
        grid.info.origin = Pose()
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = np.array(data, dtype=np.int8).tolist()

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Publish the grid
        self.publisher = self.create_publisher(OccupancyGrid, topic_name, qos_profile)
        self.publisher.publish(grid)

        self.get_logger().info("Published image!")


def main(args=None):
    rclpy.init(args=args)
    node = ImageToOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()