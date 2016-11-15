#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

# Publish one set of gridcells just to verify rviz
if __name__ == '__main__':
    rospy.init_node('lab3_gridCellTest_node')
    
    pub = rospy.Publisher('lab3/GridCells', GridCells, queue_size = 10)
    
    msg = GridCells(cell_width = 0.3, cell_height = 0.3)
    msg.header.frame_id = "map"
    
    # SimpleMap dimensions 37x37 at 0.3 resolution
    
    points = []
    for i in range(38):
        points.append(Point(i*0.3, i*0.3, 0))
    
    msg.cells = points
    
    while not rospy.is_shutdown():
        pub.publish(msg)
