#!/usr/bin/env python

import rospy, math, copy, numpy
from heapq import *
from Queue import PriorityQueue
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path

# Node class, basically a simplified Point object, with equivalence handling
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    # Close enough together
    def __eq__(self, other):
        global resolution
        return (abs(self.x - other.x) < (resolution * 0.5)) and (abs(self.y - other.y) < (resolution * 0.5))

# Reads in the global map, update globals, calls generateMapNodes
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data # Occupancy grid
    resolution = data.info.resolution
    mapData = data.data # Occupancy array
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    generateMapNodes()
    print data.info # Metadata

# Converts map data into Node objects for global mapNodes dictionary, nodes represent open space, run upon map change
def generateMapNodes():
    global mapNodes
    mapNodes = {}
    k = 0
    for i in range(0, height): # Height should be set to height of map
        for j in range(0, width): # Width should be set to width of map
            if (mapData[k] == 0):
                node=Node(0,0)
                node.x = (j * resolution) + offsetX + (0.5 * resolution)
                node.y = (i * resolution) + offsetY + (0.5 * resolution)
                mapNodes[k] = node
            k = k + 1
    
    # DEBUG ONLY, display nodes using path topic, rerun map server
    #cellsMap = GridCells()
    #cellsMap.header.frame_id = 'map'
    #cellsMap.cell_width = resolution
    #cellsMap.cell_height = resolution
    #for node in mapNodes.values():
    #    point=Point()
    #    point.x = node.x
    #    point.y = node.y
    #    point.z = 0
    #    cellsMap.cells.append(point)
    #pubPath.publish(cellsMap)
    
# Reads in the start pose from rviz and determines which node has been selected
def readStart(startPos):
    global startIndex
    
    startNode = Node(0,0)
    # Shift position to match up with node grid
    startNode.x = (startPos.pose.pose.position.x - (startPos.pose.pose.position.x % resolution)) + (resolution * 0.5)
    startNode.y = (startPos.pose.pose.position.y - (startPos.pose.pose.position.y % resolution)) + (resolution * 0.5)
    
    # Find corresponding node in map
    for index in mapNodes:
        if mapNodes[index] == startNode:
            startIndex = index
            print "Start recieved."
            # print startPos
            return
    print "WARNING: Start wasn't identified as a node."

# Reads in the goal pose from rviz, determines which node has been selected, and runs the aStar algorithm
def readGoal(goal):
    global goalIndex
    
    goalNode = Node(0,0)
    # Shift position to match up with node grid
    goalNode.x = (goal.pose.position.x - (goal.pose.position.x % resolution)) + (resolution * 0.5)
    goalNode.y = (goal.pose.position.y - (goal.pose.position.y % resolution)) + (resolution * 0.5)
    
    # Find corresponding node in map
    for index in mapNodes:
        if mapNodes[index] == goalNode:
            goalIndex = index
            print "Goal recieved."
            # print goal
            aStar() # Start Astar
            return
    print "WARNING: Goal wasn't identified as a node."

# The big bad A* algorithm function
def aStar():
    # Initialize
    # Global mapNodes is dictionary of Node objects, key is node index (so that other lists/loops can just refer to index)
    unexplored = copy.deepcopy(mapNodes) # Duplicate dictionary, will have nodes removed as they are added to frontier
    expanded = {} # Dictionary of nodes, will have nodes added as they are removed from frontier
    frontier = [] # heapq priority queue, Tuples, (f score, node index)
    heappush(frontier, (0, startIndex)) # Initial node
    cameFrom = {} # Key is index, data is index of precursor
    cameFrom[startIndex] = None
    costSoFar = {} # Key is index, data is g score
    costSoFar[startIndex] = 0
    
    # Current is tuple of (f score, node index), refer to index as current[1]
    while frontier: # While there are still nodes to explore
        current = heappop(frontier) # Pull new tuple from frontier
        expanded[current[1]] = mapNodes[current[1]] # Add current to expanded nodes
        
        if current[1] == goalIndex: # Check if arrived at goal, if so then done!
            # Construct and send final path and waypoints to rviz
            path = GridCells()
            path.header.frame_id = 'map'
            path.cell_width = resolution
            path.cell_height = resolution
            way = Path()
            way.header.frame_id = 'map'
            path.cells, way.poses = buildPath(goalIndex, cameFrom)
            way.poses = condensePoses(way.poses) # Remove superflouous intermediaries
            pubPath.publish(path)
            pubWay.publish(way)
            print "Path found!"
            return
        
        # Next is index of node object
        for next in getNeighbors(mapNodes[current[1]]): # For all neighbors of current
            newCost = costSoFar[current[1]] + calcDistance(mapNodes[current[1]], mapNodes[next]) # Calculate g score of next
            if (next not in costSoFar) or (newCost < costSoFar[next]): # Ensure that if we revisit a node it's only because we have a better path
                costSoFar[next] = newCost # Add next to costSoFar
                priority = newCost + calcDistance(mapNodes[goalIndex], mapNodes[next]) # Calculate f score of next
                heappush(frontier, (priority, next)) # Add next to the frontier
                unexplored.pop(next) # Remove next from unexplored
                cameFrom[next] = current[1] # Link next to current in cameFrom
        
        # Send the grids to be displayed in rviz
        publishCells(expanded, copy.deepcopy(frontier), unexplored)
    
    # Couldn't complete for some reason
    print "A* navigation failed."

# Returns all neighbors (nodes) of the specified node (super inefficient, should have linked the nodes or something, sorry)
def getNeighbors(node):
    # Generate the four potential neighbors
    potentials = (Node(node.x + resolution, node.y), 
                    Node(node.x - resolution, node.y),
                    Node(node.x, node.y + resolution),
                    Node(node.x, node.y - resolution))
    #print potentials
    neighbors = ()
    # Try to find potentials in the node list, return the valid ones
    for potential in potentials:
        for index in mapNodes:
            if mapNodes[index] == potential:
                neighbors = neighbors + (index,)
    return neighbors

# Calculates the straight line distance between the two nodes
def calcDistance(nodeA, nodeB):
    distance = math.sqrt((nodeB.x - nodeA.x)**2 + (nodeB.y - nodeA.y)**2) # Euclidian
    return distance

# Builds a Point array and a Path path from the cameFrom dictionary and the goal node index, recursive wow!
# Double return!
def buildPath(index, cameFrom):
    if index == startIndex: # Reached start node, return startPoint, end of recursion
        node = mapNodes[startIndex]
        point = Point(node.x, node.y, 0)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose = Pose(Point(node.x, node.y, 0), Quaternion(0, 0, 0, 1))
        return [point], [pose]
    # Else recurse for precursor node and add on the current point to the end
    node = mapNodes[index]
    point = Point(node.x, node.y, 0)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose = Pose(Point(node.x, node.y, 0), Quaternion(0, 0, 0, 1))
    ret1, ret2 = buildPath(cameFrom[index], cameFrom)
    return (ret1 + [point]), (ret2 + [pose])

# Removes unnecessary poses along waypoint pose array, just want corners
def condensePoses(path):
    remove = ()
    index = 1
    length = len(path)
    while index > 0 and index < (length-1): # Restrict to 1 to n-1 to avoid out of bounds
        current = path[index].pose.position
        prev = path[index-1].pose.position
        next = path[index+1].pose.position
        # All three colinear
        if (((abs(prev.x - current.x) < (resolution * 0.5)) and (abs(current.x - next.x) < (resolution * 0.5))) or
                ((abs(prev.y - current.y) < (resolution * 0.5)) and (abs(current.y - next.y) < (resolution * 0.5)))):
            path.pop(index) # Remove the middle pose, unnecessary
            length = len(path) # Refresh list length
            # Keep index the same, since list has shifted back
        else:
            index = index + 1 # Increment
        
        # Current is a corner, should alter orientation to look to next node
        #elif (((abs(prev.x - current.x) < (resolution * 0.5)) and (abs(current.y - next.y) < (resolution * 0.5))) or
        #        ((abs(prev.y - current.y) < (resolution * 0.5)) and (abs(current.x - next.x) < (resolution * 0.5)))):
        #    pass # Do this next lab I guess
    return path

# Publishes three occupancy grids (expanded, frontier, and unexplored) from given lists to rviz using gridcells type
# Expanded is dictionary or nodes, frontier is heapq priority queue, unexplored is dictionary of nodes
def publishCells(expanded, frontier, unexplored):
    global pubExpanded
    global pubFrontier
    global pubUnexplored
    
    # Same structure for all three, base GridCells off of global map, translate node to point

    # Expanded
    cellsExpanded = GridCells()
    cellsExpanded.header.frame_id = 'map'
    cellsExpanded.cell_width = resolution
    cellsExpanded.cell_height = resolution
    for node in expanded.values():
        point=Point()
        point.x = node.x
        point.y = node.y
        point.z = 0
        cellsExpanded.cells.append(point)
    pubExpanded.publish(cellsExpanded)
    
    # Frontier
    cellsFrontier = GridCells()
    cellsFrontier.header.frame_id = 'map'
    cellsFrontier.cell_width = resolution
    cellsFrontier.cell_height = resolution
    for element in frontier:
        node = mapNodes[element[1]]
        point=Point()
        point.x = node.x
        point.y = node.y
        point.z = 0
        cellsFrontier.cells.append(point)
    pubFrontier.publish(cellsFrontier)
    
    # Unexplored
    cellsUnexplored = GridCells()
    cellsUnexplored.header.frame_id = 'map'
    cellsUnexplored.cell_width = resolution
    cellsUnexplored.cell_height = resolution
    for node in unexplored.values():
        point=Point()
        point.x = node.x
        point.y = node.y
        point.z = 0
        cellsUnexplored.cells.append(point)
    pubUnexplored.publish(cellsUnexplored)

# Main handler of the project
def run():
    global pubExpanded
    global pubFrontier
    global pubUnexplored
    global pubPath
    global pubWay
    
    rospy.init_node('lab3_aStar_navigation_node')
    
    # Publishers and subscribers
    subMap = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pubExpanded = rospy.Publisher("/lab3/expandedCells", GridCells, queue_size=1)
    pubFrontier = rospy.Publisher("/lab3/frontierCells", GridCells, queue_size=1)
    pubUnexplored = rospy.Publisher("/lab3/unexploredCells", GridCells, queue_size=1)
    pubPath = rospy.Publisher("/lab3/path", GridCells, queue_size=1)
    pubWay = rospy.Publisher("/lab3/waypoints", Path, queue_size=1)
    subGoal = rospy.Subscriber('/lab3/goalPose', PoseStamped, readGoal, queue_size=1)
    subStart = rospy.Subscriber('/lab3/startPose', PoseWithCovarianceStamped, readStart, queue_size=1)

    # Wait around while A*
    rospy.spin()
    

# Main function
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
