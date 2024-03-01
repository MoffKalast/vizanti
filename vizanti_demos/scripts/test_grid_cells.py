#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
import random
import time

class RandomGridCellsPublisher(Node):
    def __init__(self):
        super().__init__('test_grid_cells')
        self.publisher_ = self.create_publisher(GridCells, 'test_grid_cells', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.i % 10 == 0:
            # Generate a random resolution between 0.1 and 1.0
            resolution_x = random.uniform(0.1, 3)+0.01
            resolution_y = random.uniform(0.1, 3)+0.01

            # Generate a random number of cells
            num_cells = random.randint(1, 500)

            # Generate random cells
            cells = GridCells()
            cells.header.stamp = self.get_clock().now().to_msg()
            cells.header.frame_id = 'test_link'
            cells.cell_width = resolution_x
            cells.cell_height = resolution_y

            for _ in range(num_cells):
                point = Point()
                point.x = resolution_x * random.randint(-80, 80)
                point.y = resolution_y * random.randint(-80, 80)
                point.z = 0.0
                cells.cells.append(point)
            
            self.publisher_.publish(cells)
        elif self.i % 10 == 9:
            # Publish an empty GridCells message
            empty_cells = GridCells()
            empty_cells.header.stamp = self.get_clock().now().to_msg()
            empty_cells.header.frame_id = 'test_link'
            empty_cells.cell_width = 1.0
            empty_cells.cell_height = 1.0
            self.publisher_.publish(empty_cells)

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    random_grid_cells_publisher = RandomGridCellsPublisher()
    rclpy.spin(random_grid_cells_publisher)
    random_grid_cells_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()