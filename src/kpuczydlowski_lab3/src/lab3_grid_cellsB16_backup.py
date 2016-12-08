#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math
import rospy, tf, numpy, math
import math
from Queue import PriorityQueue

class Node(object):

    # val is the number of steps from start
    # coor - coordinate in (x,y) tuple form
    # parent - node object
    # start - coordinate tuple
    # goal - coordinate tuple
    # path - list of vals
    def __init__(self, val, parent, start, goal, coor):
        self.children = []
        self.parent = parent
        self.val = val
        self.coor = coor
        if parent: #parent != None
            self.path = parent.path[:]
            self.path.append(self.coor)
            self.start = parent.start
            self.goal = parent.goal
        else:
            self.path = [self.coor]
            self.start = start
            self.goal = goal

        self.dist = self.getEucDist()

    def getManDist(self):
        return abs(float(self.coor[0]) - self.goal[0]) + abs(float(self.coor[1]) - self.goal[1])

    def getEucDist(self):
        return math.sqrt(float(self.coor[0] - self.goal[0])*(self.coor[0] - self.goal[0]) + float(self.coor[1] - self.goal[1])*(self.coor[1] - self.goal[1]))

    def getCost(self):
        pass

        # generate possible children
        # dont generate impossible children
    def generateChildren(self):
        global mapData
        global width
        #check if already made children
        if not self.children:

            #case [x-1,y]
            index = int( ((self.coor[0]- 1) + (self.coor[1] * width)) )
            #print index
            v = self.val + 1
            if(mapData[index] != 100):
                temp = ((self.coor[0]-1), self.coor[1]);
                baby = Node(v, self, self.start, self.goal, temp)
                self.children.append(baby)
            #case [x+1,y]
            index = int((self.coor[0]+ 1) + (self.coor[1] * width))
            if(mapData[index] != 100):
                temp = ((self.coor[0]+1), self.coor[1]);
                baby = Node(v, self, self.start, self.goal, temp)
                self.children.append(baby)

            #case [x,y-1]
            index = int(self.coor[0] + ( (self.coor[1]-1) * width))
            if(mapData[index] != 100):
                temp = (self.coor[0], (self.coor[1]-1));
                baby = Node(v, self, self.start, self.goal, temp)
                self.children.append(baby)

            #case [x,y+1]
            index = int(self.coor[0] + ( (self.coor[1]+1) * width))
            if(mapData[index] != 100):
                temp = (self.coor[0], (self.coor[1]+1));
                baby = Node(v, self, self.start, self.goal, temp)
                self.children.append(baby)

def runAStar():
    global startPosX
    global startPosY
    global goalX
    global goalY
    gc = (goalX, goalY);
    coor = (startPosX, startPosY);
    start = Node(0, None, coor, gc, coor)
    path = []
    visited = []
    pqueue = PriorityQueue()
    count = 0
    pqueue.put((0,count,start))
    while(not path and pqueue.qsize()):
        closest = pqueue.get()[2]
        closest.generateChildren()
        visited.append(closest.coor)
        for c in closest.children:
            #print coor
            if c.coor not in visited:
                count += 1
                if not c.dist:
                    path = c.path
                    break
                pqueue.put((c.dist,count,c))
    if not path:
        print "no path"
    for p in path:
        print p
    return path


# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def readGoal(goal):
    global goalX
    global goalY
    print 'readGoal'
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print goal.pose
    runAStar()
    pubPathCells()


def readStart(startPos):
    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print 'readStart'
    print startPos.pose.pose

def pubPathCells(tlist):
    global pubpath
    print "publishing path"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(len(tlist)):
        point=Point()
        point.x=(tlist[i][0]*resolution)+offsetX + (1.5 * resolution) # added secondary offset
        point.y=(tlist[i][1]*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
        point.z = 0
        cells.cells.append(point)

    pubpath.publish(cells)


def aStar(start,goal):

    runAStar()
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points

#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
    pub.publish(cells)

#Main handler of the project
def run():
    global pub
    global pubpath
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('/move_base_simple/goal2', PoseStamped, readGoal, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(10)

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)
        print("Complete")



if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
