#!/usr/bin/env python

import rospy
import random
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

def generate_random_cells(count=100, x_range=(-100, 100), y_range=(-100, 100)):
    points = []
    for _ in range(count):
        pt = Point()
        pt.x = random.uniform(*x_range)
        pt.y = random.uniform(*y_range)
        pt.z = 0  # GridCells are generally 2D
        points.append(pt)
    return points

def publish_gridcells():
    pub = rospy.Publisher('/test_gridcells', GridCells, queue_size=1, latch=True)
    rospy.init_node('gridcell_publisher', anonymous=True)
    rate = rospy.Rate(1.0 / 15.0)  # Publish every 30 seconds

    while not rospy.is_shutdown():
        grid = GridCells()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "local"  # Change to match your TF frame
        grid.cell_width = 1
        grid.cell_height = 1
        grid.cells = generate_random_cells()
        pub.publish(grid)
        rospy.loginfo("Published GridCells with %d cells", len(grid.cells))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_gridcells()
    except rospy.ROSInterruptException:
        pass
