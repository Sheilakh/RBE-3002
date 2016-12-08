#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import numpy
import math
from Queue import PriorityQueue
import Robot

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])*(p0[0] - p1[0]) + (p0[1] - p1[1])*(p0[1] - p1[1]))

# reads in global map
def mapCallBack(data):
    print "map callback"
    # data = optimizeMap(data)
    mapgrid = data
    self.res = data.info.self.res
    self.mapdata = data.data
    self.w = data.info.width
    self.h = data.info.height
    self.ox = data.info.origin.position.x
    self.oy = data.info.origin.position.y

    error = 1.25 # Margin to add to robot radius
    self.expR = math.ceil((0.12 * error) / self.res)

    if state == astarstate:
        print 'checking astar'
        if(self.checkOccupancy(mapData, astarpath, self.w, 99)):
            print 'stopping'
            self.pubTwist(0, 0)
            astarpath = runAStar()
            pubPathCells(astarpath)
            c = getWaypointTuple(astarpath)
            executeTraj(c)

    print data.info

def timerCallback(data):
    global pose

def main():
	rospy.init_node('lab5')
	velpub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size =2)
	mapsub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	wallpub = rospy.Publisher("/walls", GridCells, queue_size=1)
	astarpub = rospy.Publisher("/astarpath", GridCells, queue_size=1)
	waypointpub = rospy.Publisher("/wps", GridCells, queue_size=1)
	goalsub = rospy.Subscriber('/move_base_simple/goal2', PoseStamped, readGoal, queue_size=1) #change topic for best results
    global odom_list
    odom_list = tf.TransformListener()
    global pose
    rospy.Timer(rospy.Duration(0.1), timerCallback)

    robot = Robot()


if __name__ == '__main__':
    main()
