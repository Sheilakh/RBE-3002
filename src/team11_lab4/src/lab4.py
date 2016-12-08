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

# Drive directly to a pose
def navToPose(goal):
	global pose
	global obsDetected
	goalPose = goal.pose

	# Calculate the angle and distance to the goal
	deltaX = goalPose.position.x - pose.position.x
	deltaY = goalPose.position.y - pose.position.y
	theta = math.degrees(math.atan2(deltaY, deltaX)) # Global frame
	dist = math.sqrt(deltaX**2 + deltaY**2)

	# Convert goal orientation from quaternion to a yaw angle
	quat = (
		goalPose.orientation.x,
		goalPose.orientation.y,
		goalPose.orientation.z,
		goalPose.orientation.w)
	euler = euler_from_quaternion(quat)
	goalAngle = math.degrees(euler[2]) # Yaw

	# Rotate until facing the goal
	print "spin!"
	if not rotateTo(theta, 0.4):
		print "obs"
		return False
	rospy.sleep(2)

	# Drive straight until on goal
	print "move!"
	if not driveStraight(0.1, dist): # If halted due to obstacle
		print "Obs"
		return False
	rospy.sleep(0.5)
	
	# Use built-in navigation stack instead
	#rate = rospy.Rate(100) # 100hz
	#while(math.sqrt((goalPose.position.x - pose.position.x)**2 + (goalPose.position.x - pose.position.x)**2) > 0.1):
	#	goalpub.publish(goal)
	#	sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	#	if (obsDetected):
	#		return False
	#	rate.sleep()

	# Rotate until facing proper direction
	#print "spin!"
	#rotateTo(goalAngle, 0.2)
	#rospy.sleep(0.5)

	print "done"
	return True

# This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pose
	global obsDetected
	# Record initial position
	initX = pose.position.x
	initY = pose.position.y
	atTarget = False

	# Drive (twist(speed,0)) until distance has passed, based on current and initial positions
	while not atTarget and not rospy.is_shutdown():
		# Check if obstacle detected, if so then kill movement and return
		if obsDetected:
			print "HALT"			
			pubTwist(0,0)
			obsDetected = False
			return False
		
		if not (pose.position.x == 0 and pose.position.y == 0): # Account for occasional 0 position
			currentX = pose.position.x
			currentY = pose.position.y
		currentDist = math.sqrt((currentX - initX)**2 + (currentY - initY)**2)
		if currentDist >= distance:
			atTarget = True
			pubTwist(0,0)
		else:
			pubTwist(speed,0)
			rospy.sleep(0.15) # In lab2 this was needed, but not anymore?

	# Stop
	pubTwist(0,0)
	
	return True

#Accepts a relative angle and makes the robot rotate by that much. (-180 to 180)
def rotate(theta, speed):
	global pose
	global obsDetected

	if theta < -180 or theta > 180:
		print "Keep angle within -180 to 180 please."

	# Record initial orientation
	initAng = math.degrees(pose.orientation.z)
	atTarget = False

	# Make sure rotating by smallest angle
	if theta > 180:
		theta = -360 + theta
	elif theta < -180:
		theta = 360 + theta

	# Determine rotation direction
	if theta > 0:
		angVel = speed
	elif theta < 0:
		angVel = -speed
	else:
		atTarget = True

	# Rotate (twist(0,speed)) until proper angle reached, based on current and initial rotations
	while not atTarget and not rospy.is_shutdown():
		currentAng = math.degrees(pose.orientation.z)
		traveled = currentAng - initAng
		if obsDetected:
			print "HALT"
			pubTwist(0,0)
			return False
		# Account for wraparound
		if traveled > 180:
			traveled = -360 + traveled
		elif traveled < -180:
			traveled = 360 + traveled

		if abs(traveled) > abs(theta):
			atTarget = True
			pubTwist(0,0)
		else:
			pubTwist(0,angVel)
			rospy.sleep(0.15) # In lab2 this was needed, but not anymore?

	# Stop
	pubTwist(0,0)
	return True

# Accepts a global angle and makes the robot rotate to it. (-180 to 180)
def rotateTo(angle, speed):
	global pose
	global obsDetected
	initAng = math.degrees(pose.orientation.z)
	theta = angle - initAng
	# Make sure rotating by smallest angle
	if theta > 180:
		theta = -360 + theta
	elif theta < -180:
		theta = 360 + theta
	return rotate(theta, speed)

def pointToGrid(pt):
	global resolution
	global offsetX
	global offsetY
	gridx = int((pt[0]+ (.5*resolution) - offsetX)/resolution)
	gridy = int((pt[1]+ (.5*resolution) - offsetY)/resolution)
	r = (gridx, gridy);
	return r

def gridToPoint(grid):
	global resolution
	global offsetX
	global offsetY
	x=(grid[0]*resolution)+offsetX - (.5 * resolution)
	y=(grid[1]*resolution)+offsetY - (.5 * resolution)
	r = (x,y);
	return r

# To make publishing easier, just give linear and angular velocity
def pubTwist(lin, ang):
	global velpub
	msg = Twist()
	msg.linear.x = lin
	msg.angular.z = ang
	velpub.publish(msg)

def executeTraj(plist):
	global pose
	global pubway
	global width
	global resolution
	global astarpath
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution
	cells.cell_height = resolution

	# print poses
	print 'print poses'
	print 'global pose'
	print pose
	print pointToGrid((pose.position.x,pose.position.y))
	for p in range(len(plist)):
		point = Point()
		print 'Way Point ' + str(p)
		pt = gridToPoint(plist[p])
		point.x = pt[0]
		point.y = pt[1]
		nextpose = PoseStamped()
		nextpose.pose.position.x = pt[0]
		nextpose.pose.position.y = pt[1]
		nextpose.pose.orientation.z = pose.orientation.z
		print nextpose.pose
		print plist[p]
		print
		cells.cells.append(point)
	#show waypoints as grid cells
	pubway.publish(cells)

	#travel to waypoints
	count = 1
	while(count < len(plist)):
		nextpose = PoseStamped()
		nextpose.header.stamp = rospy.Time().now()
		nextpose.header.frame_id = 'map'
		pt = gridToPoint(plist[count])
		nextpose.pose.position.x = pt[0]
		nextpose.pose.position.y = pt[1]
		nextpose.pose.orientation.z = pose.orientation.z
		b = navToPose(nextpose)
		if(not b):
			print "Rerunning aStar"
			astarpath = runAStar()
			pubPathCells(astarpath)
			isAstar = True
			plist = getWaypointTuple(astarpath)
			count = 1
		else:
			count += 1

# Return a list of tuples corresponding to
# grid cell coordinates of waypoints
def getWaypointTuple(tlist):
	if not len(tlist):
		return ()

	poseList = []
	numsq = 0
	direc = 0
	prev = -1
	difx = tlist[1][0] - tlist[0][0]
	dify = tlist[1][1] - tlist[0][1]
	poseList.append(tlist[0])
	#get first angle
	if(difx == -1 and dify ==0):
		prev = 0
	elif(difx == 0 and dify ==-1):
		prev = 1
	elif(difx == 1 and dify ==0):
		prev = 2
	elif(difx == 0 and dify ==1):
		prev = 3
	elif(difx == 1 and dify ==1):
		prev = 4
	elif(difx == 1 and dify ==-1):
		prev = 5
	elif(difx == -1 and dify ==1):
		prev = 6
	elif(difx == -1 and dify ==-1):
		prev = 7

	for i in xrange(2,len(tlist)-1):
		difx = tlist[i][0] - tlist[i-1][0]
		dify = tlist[i][1] - tlist[i-1][1]
		if(difx == -1 and dify ==0):
			direc = 0
		elif(difx == 0 and dify ==-1):
			direc = 1
		elif(difx == 1 and dify ==0):
			direc = 2
		elif(difx == 0 and dify ==1):
			direc = 3
		elif(difx == 1 and dify ==1):
			direc = 4
		elif(difx == 1 and dify ==-1):
			direc = 5
		elif(difx == -1 and dify ==1):
			direc = 6
		elif(difx == -1 and dify ==-1):
			direc = 7
		if(prev == direc):
			numsq += 1
		else:
			poseList.append(tlist[i-1])
		prev = direc
	poseList.append(tlist[-1])
	return poseList

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

		self.dist = self.getCost()

	def getManDist(self):
		return abs(float(self.coor[0]) - self.goal[0]) + abs(float(self.coor[1]) - self.goal[1])

	def getEucDist(self):
		return math.sqrt(float(self.coor[0] - self.goal[0])*(self.coor[0] - self.goal[0]) + float(self.coor[1] - self.goal[1])*(self.coor[1] - self.goal[1]))

	def getCost(self):
		return self.getManDist() + self.val

		# dont generate impossible children
	def generateChildren(self):
		global mapData
		global width
		global expRadius
		#check if already made children
		#data = wallExpansion(mapData, width, expRadius, 99)

		# Reset children
		self.children = []

		if not self.children:

			v = self.val + 1
			#case [x-1,y]
			index = int( ((self.coor[0]- 1) + (self.coor[1] * width)) )
			if(index > 0):
				if (mapData[index] != 100):
					temp = ((self.coor[0]-1), self.coor[1]);
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x+1,y]
			index = int((self.coor[0]+ 1) + (self.coor[1] * width))
			if(index < len(mapData)):
				if(mapData[index] != 100):
					temp = ((self.coor[0]+1), self.coor[1]);
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)

			#case [x,y-1]
			index = int(self.coor[0] + ( (self.coor[1]-1) * width))
			if(index > 0):
				if(mapData[index] != 100):
					temp = (self.coor[0], (self.coor[1]-1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)

			#case [x,y+1]
			index = int(self.coor[0] + ( (self.coor[1]+1) * width))
			if(index < len(mapData)):
				if(mapData[index] != 100):
					temp = (self.coor[0], (self.coor[1]+1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x-1,y-1]
			index = int(self.coor[0]-1 + ( (self.coor[1]-1) * width))
			if(index > 0):
				if(mapData[index] != 100):
					temp = (self.coor[0]-1, (self.coor[1]-1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x-1,y+1]
			index = int(self.coor[0]-1 + ( (self.coor[1]+1) * width))
			if(index > 0 and index < len(mapData)):
				if(mapData[index] != 100):
					temp = (self.coor[0]-1, (self.coor[1]+1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x+1,y-1]
			index = int(self.coor[0]+1 + ( (self.coor[1]-1) * width))
			if(index > 0 and index < len(mapData)):
				if(mapData[index] != 100):
					temp = (self.coor[0]+1, (self.coor[1]-1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			 #case [x+1,y+1]
			index = int(self.coor[0]+1 + ( (self.coor[1]+1) * width))
			if(index > 0 and index < len(mapData)):
				if(mapData[index] != 100):
					temp = (self.coor[0]+1, (self.coor[1]+1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)

# expand the walls of a given mapData
# to a given radius
def wallExpansion(data, offset, radius, thresh):
	newData =[]
	for i in data:
		newData.append(int(i))
	checked = []
	index = 0
	iterations = 0
	while(iterations < radius):
		while(index < len(data)):
			if(index not in checked and data[index] > thresh):
				x = index%offset
				y = int(index/offset)
				#case [x-1,y]
				cur = int( ((x-1) + (y * offset)) )
				if(cur > 0 and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
				cur = int( ((x+1) + (y * offset)) )
				if(cur < len(data) and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
				cur = int( (x + ((y-1) * offset)) )
				if(cur > 0 and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
				cur = int( (x + ((y+1) * offset)) )
				if(cur < len(data) and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)

				cur = int( ((x-1) + ((y-1) * offset)) )
				if(cur > 0 and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
				cur = int( ((x+1) + ((y-1) * offset)) )
				if(cur > 0 and cur < len(data) and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
				cur = int( ((x-1) + ((y+1) * offset)) )
				if(cur > 0 and cur < len(data) and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
				cur = int( ((x+1) + ((y+1) * offset)) )
				if(cur < len(data) and newData[cur] < data[index]):
					newData[cur] = data[index]
					checked.append(cur)
			index += 1
		index = 0
		iterations += 1
		data = newData
		checked = []
	return newData

# run A star search
def runAStar():
	global pose
	startPosX = pose.position.x
	startPosY = pose.position.y
	global goalX
	global goalY

	sp = pointToGrid((startPosX, startPosY))
	gp = pointToGrid((goalX, goalY))

	start = Node(0, None, sp, gp, sp)
	path = []
	visited = []
	pqueue = PriorityQueue()
	count = 0
	pqueue.put((start.dist,start))
	print"Starting A-star search"

	while(not path and pqueue.qsize()):
		closest = pqueue.get()[1]
		closest.generateChildren()
		visited.append(closest.coor)
		for c in closest.children:
			if c.coor not in visited:
				count += 1
				if not c.getManDist():
					path = c.path
					break
				pqueue.put((c.dist,c))
	if not path:
		print "no path"
	else:
		print "found path"
	for p in path:
		print p
	print
	return path

def checkOccupancy(newdata, astarp, offset, thresh):
	if newdata[astarp[0][0] + offset*astarp[0][1]]:
		print "here"
		return False
	for a in astarp:
		index = a[0] + offset*a[1]
		print "checking occupancy"
		#print a
		#print newdata[index]
		if((newdata[index] >= thresh)):
			return False
	return True

# reads in global map
def mapCallBack(data):
	global mapData
	global width
	global height
	global mapgrid
	global resolution
	global offsetX
	global offsetY
	global isAstar
	global expRadius
	global astarpath
	global goalpub
	global pose
	global obsDetected
	#print "map callback"
	#if(width!=data.info.width):
	#	print data.info.width
	# data = optimizeMap(data)
	mapgrid = data
	resolution = data.info.resolution
	mapData = data.data
	width = data.info.width
	height = data.info.height
	offsetX = data.info.origin.position.x
	offsetY = data.info.origin.position.y

	error = 1.5 # Margin to add to robot radius
	expRadius = math.ceil((0.12 * error) / resolution)
	mapData = wallExpansion(mapData, width, expRadius, 99)
	publishCells(mapData)
	print "Walls expanded"
	if isAstar:
		print 'Rechecking astar'
		#data = wallExpansion(mapData, width, expRadius, 99)
		if(not checkOccupancy(mapData,astarpath,width,99)):
			print 'stopping'
			obsDetected = True
			isAstar = False
			#ps = PoseStamped()
			#ps.header.stamp = rospy.Time().now()
			#ps.header.frame_id = 'map'
			#ps.pose.position.x = pose.position.x
			#ps.pose.position.y = pose.position.y
			#ps.pose.orientation.z = pose.orientation.z
			# goalpub.publish(ps)

			#c = getWaypointTuple(astarpath)
			#executeTraj(c)
		else:
			obsDetected = False

	#print data.info

# Expands map resolution to speed up aStar
def optimizeMap(data):
	map0 = data.data
	resolution0 = data.info.resolution
	width0 = data.info.width
	height0 = data.info.width

	resolution1 = 0.23 / 4 # Based on wheelbase
	scale = resolution1 / resolution0
	width1 = int((width0 * scale) + 1)
	height1 = int((height0 * scale) + 1)

	map1 = [int(0)]*(width1 * height1)
	index = 0
	while(index < len(map0)):
		if map0[index] == 100:
			map1[int(index/scale)] = 100
		index += 1

	newData = data
	newData.data = map1
	newData.info.resolution = resolution1
	newData.info.width = width1
	newData.info.height = height1
	return newData

# Goal callback function
# set the global goal variables
# and execute the trajectory using A-star search
def readGoal(goal):
	global goalX
	global goalY
	global isAstar
	global astarpath

	print 'readGoal'
	goalX= goal.pose.position.x
	goalY= goal.pose.position.y
	print goal.pose
	print pointToGrid((goal.pose.position.x,goal.pose.position.y))
	print
	astarpath = runAStar()
	isAstar = True
	pubPathCells(astarpath)
	c = getWaypointTuple(astarpath)
	executeTraj(c)

# publish the astar path
def pubPathCells(tlist):
	global pubpath
	print "Publishing A-Star Path"

	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution
	cells.cell_height = resolution
	for i in range(len(tlist)):
		point=Point()
		pt = gridToPoint((tlist[i][0], tlist[i][1]))
		point.x= pt[0]
		point.y= pt[1]
		point.z = 0
		cells.cells.append(point)
	pubpath.publish(cells)


#publishes map to rviz using gridcells type
def publishCells(grid):
	global pub
	global width
	global height
	global resolution
	global expRadius
	# resolution and offset of the map
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution
	cells.cell_height = resolution
	k=0
	#grid = wallExpansion(grid, width, expRadius, 99)
	for i in range(1,height+1): #height should be set to hieght of grid
		for j in range(1,width+1): #width should be set to width of grid
			if (grid[k] == 100):
				point=Point()
				pt = gridToPoint((j,i))
				point.x= pt[0]
				point.y= pt[1]
				point.z=0
				cells.cells.append(point)
			k=k+1
	pub.publish(cells)

# Timer callback function
# read odometry data
def timerCallback(event):
	global pose
	global odom_list
	#sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	odom_list.waitForTransform("/map","/base_footprint",rospy.Time(0),rospy.Duration(1, 0))
	c_p, c_q = odom_list.lookupTransform("/map", "/base_footprint", rospy.Time(0))
	pose.position.x = c_p[0]
	pose.position.y = c_p[1]
	q = [c_q[0],c_q[1],c_q[2],c_q[3]]
	rol, pit, yaw = euler_from_quaternion(q)
	pose.orientation.z = yaw

def timerCallback2(event):
	global mapData
	#publishCells(mapData)

#Main handler of the project
def run():
	global velpub
	global pub
	global pub2
	global pubpath
	global pose
	global odom_list
	global pubway
	global isAstar
	global astarpath
	global goalpub
	global obsDetected
	obsDetected = False
	isAstar = False
	rospy.init_node('lab4')

	pose = Pose()
	velpub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size =2)
	sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
	pub2 = rospy.Publisher("/astar", GridCells, queue_size=1)
	pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
	pubway = rospy.Publisher("/wps", GridCells, queue_size=1)
	start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
	goal_sub = rospy.Subscriber('/move_base_simple/goal2', PoseStamped, readGoal, queue_size=1) #change topic for best results
	goalpub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
	odom_list = tf.TransformListener()
	# wait a second for publisher, subscribers, and TF
	rospy.sleep(2)
	rospy.Timer(rospy.Duration(0.05), timerCallback)
	rospy.Timer(rospy.Duration(0.15), timerCallback2)

	while (not rospy.is_shutdown()):
		#publishCells(mapData) #publishing map data every 2 seconds

		#rospy.sleep(0.1)
		rospy.spin()		
		#print("Complete")


# read in start position from /initialpose topic
def readStart(startPos):
	global startPosX
	global startPosY
	global pub2
	startPosX = startPos.pose.pose.position.x
	startPosY = startPos.pose.pose.position.y
	print 'readStart'
	print startPos.pose.pose
	print pointToGrid((startPos.pose.pose.position.x,startPos.pose.pose.position.y))
	print
	sp = (startPosX, startPosY);
	sp = pointToGrid(sp)
	# resolution and offset of the map
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution
	cells.cell_height = resolution
	point=Point()
	sp = gridToPoint((sp[0],sp[1]))
	point.x= sp[0]
	point.y= sp[1]
	point.z = 0
	cells.cells.append(point)
	pub2.publish(cells)



if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
