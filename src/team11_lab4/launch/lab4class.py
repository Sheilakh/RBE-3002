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

class Robot(object):

    def __init__(self, velpub, pathpub, mappub, goalsub, mapsub):
        self.pose = Pose()
        self.x = 0
        self.y = 0
        self.z = 0
        self.tx = 0
        self.ty = 0
        self.tz = 0
        self.traj_step = 2
        self.tempgoalset = False
        self.mapdata = []
        self.gx = -1
        self.gy = -1
        self.odom_list = tf.TransformListener
        self.state = -1
        self.astarpath = []
        self.waypoints = []
        self.w = 0
        self.h = 0
        self.ox = 0
        self.oy = 0
        self.res = 0.0
        self.expR = 0.0
        self.velpub = velpub
        self.pathpub = pathpub
        self.mappub = mappub
        self.goalsub = goalsub
        self.mapsub = mapsub

    # Goal callback function
    # set the global goal variables
    # and execute the trajectory using A-star search
    def readGoal(self, goal):
    	print 'readGoal'
    	self.gx = goal.pose.position.x
    	self.gy = goal.pose.position.y
    	print goal.pose
    	print pointToGrid((goal.pose.position.x,goal.pose.position.y))
    	print
    	self.state = FIGUREOUTSTATE

    def timerCallback(self, e):
        self.odom_list.waitForTransform("/map","/base_footprint",rospy.Time(0),rospy.Duration(1, 0))
    	c_p, c_q = odom_list.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    	self.x = c_p[0]
        self.y = c_p[1]
        q = [c_q[0],c_q[1],c_q[2],c_q[3]]
    	rol, pit, yaw = euler_from_quaternion(q)
    	self.z = math.degrees(yaw)

    def checkOccupancy(self, newdata, astarp ,offset, thresh):
    	for a in astarp:
    		index = a[0] + offset*a[1]
    		if(newdata[index] >= thresh):
    			return True
    	return False

    # expand the walls of a given mapData
    # to a given radius
    def wallExpansion(self, data, offset, radius, thresh):
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
    def runAStar(self, startpose, goalpose):
    	sp = pointToGrid((startpose.position.x, startpose.position.y))
    	gp = pointToGrid((goalpose.position.x, goalpose.position.y))
    	start = Node(0, None, sp, gp, sp)
    	path = []
    	visited = []
    	pqueue = PriorityQueue()
    	count = 0
    	pqueue.put((0,start))
    	print"Starting A-star search"

    	while(not path and pqueue.qsize()):
    		closest = pqueue.get()[2]
    		closest.generateChildren()
    		visited.append(closest.coor)
    		for c in closest.children:
    			if c.coor not in visited:
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

    # This function accepts a speed and a distance for the robot to move in a straight line
    # returns whether or not the robot has completed that distance
    def driveStraight(self, speed_, dist_):
    	# Record initial position
        if(not self.tempgoalset):
            self.tx = self.x + dist_*math.cos(self.z)
            self.ty = self.y + dist_*math.sin(self.z)
            self.tempgoalset = True
        #distance between current pose and drive goal
        dist = distance((self.x, self.y),(self.tx,self.ty))
    	atTarget = dist < 0.02

    	# Drive (twist(speed,0)) until distance has passed, based on current and initial positions
    	if not rospy.is_shutdown():
            if atTarget:
    			self.pubTwist(0,0)
                self.tempgoalset = False
                return 0
    		else:
                kp = 0.75
                if(dist/dist_ < 0.5):
                    speed = (dist/dist_)*kp*speed_
                else:
                    speed = ((dist_-dist)/dist_)*kp*speed_
                if(speed < 0.1):
                    speed = 0.1
		        self.pubTwist(speed,0)
                return distance((self.x, self.y),(self.tx,self.ty))

         return -1
    #Accepts a relative angle and makes the robot rotate by that much. (-180 to 180)
    def rotate(self, theta, speed):

    	if theta < -180 or theta > 180:
    		print "Keep angle within -180 to 180 please."

    	# Make sure rotating by smallest angle
    	if theta > 180:
    		theta = -360 + theta
    	elif theta < -180:
    		theta = 360 + theta

    	# Determine rotation direction
    	if theta > 0:
    		speed = speed
    	elif theta < 0:
    		speed = -speed

    	# Record initial position
        if(not self.tempgoalset):
            self.tz = self.z+theta
            self.tempgoalset = True
        traveled =  self.tz - self.z

        # Account for wraparound
		if traveled > 180:
			traveled = -360 + traveled
		elif traveled < -180:
			traveled = 360 + traveled

        atTarget = abs(traveled) >= abs(theta)
    	# Drive (twist(speed,0)) until distance has passed, based on current and initial positions
    	if not rospy.is_shutdown():
            if atTarget:
    			self.pubTwist(0,0)
                self.tempgoalset = False
                return 0
    		else:
                kp = 0.75
                if(dist/dist_ < 0.5):
                    speed = (traveled/theta)*kp*speed_
                else:
                    speed = ((theta-traveled)/theta)*kp*speed_
                if(speed < 0.1):
                    speed = 0.1
		        self.pubTwist(0,speed)
                return traveled

        return -1

    # Drive directly to a pose
    def navToPose(self, goal):

        # Calculate the angle and distance to the goal
        deltaX = goal.pose.position.x - self.x
        deltaY = goal.pose.position.y - self.y
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
        if(self.traj_step%2 == 0):
            d = self.rotate(theta, 0.5)
            if (d != 0):
                return (1, d)
            else:
                self.traj_step += 1
                return (1,0)
        else:
            d = self.driveStraight(0.2, dist)
            if(d != 0):
                return (2,d)
            else:
                self.traj_step += 1
                return (2,0)


    def executeTraj(self, plist):
        i = int(self.traj_step/2)
		nextpose = PoseStamped()
		pt = gridToPoint(plist[i])
		nextpose.pose.position.x = pt[0]
		nextpose.pose.position.y = pt[1]
		nextpose.pose.orientation.z = pose.orientation.z
        termtest = navToPose(nextpose)
        if (termtest == (2,0)):
            if (i == len(plist)-1):
                return 0

        return termtest[1]

    	#travel to waypoints
    	for p in xrange(1,len(plist)):
    		nextpose = PoseStamped()
    		pt = gridToPoint(plist[p])
    		nextpose.pose.position.x = pt[0]
    		nextpose.pose.position.y = pt[1]
    		nextpose.pose.orientation.z = pose.orientation.z
    		navToPose(nextpose)

    def scan(self):
        if(self.rotate(180,0.5) == 0):
            return 1
        return 0

    # publish the astar path
    def pubPathCells(self, tlist):
    	print "Publishing A-Star Path"

    	cells = GridCells()
    	cells.header.frame_id = 'map'
    	cells.cell_width = self.res
    	cells.cell_height = self.res
    	for i in range(len(tlist)):
    		point=Point()
    		pt = gridToPoint((tlist[i][0],tlist[i][1]) )
    		point.x= pt[0]
    		point.y= pt[1]
    		point.z = 0
    		cells.cells.append(point)
    	self.pathpub.publish(cells)

    # Return a list of tuples corresponding to
    # grid cell coordinates of waypoints
    def getWaypointTuple(self, tlist):

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

    #publishes map to rviz using gridcells type
    def publishCells(self, grid):
    	# self.res and offset of the map
    	cells = GridCells()
    	cells.header.frame_id = 'map'
    	cells.cell_width = self.res
    	cells.cell_height = self.res
    	k=0
    	grid = self.wallExpansion(grid, self.w, self.expR, 99)
    	for i in range(1,self.h+1): #height should be set to hieght of grid
    		for j in range(1,self.w+1): #width should be set to width of grid
    			if (grid[k] == 100):
    				point=Point()
    				pt = gridToPoint((j,i))
    				point.x= pt[0]
    				point.y= pt[1]
    				point.z=0
    				cells.cells.append(point)
    			k=k+1
    	self.mappub.publish(cells)

    # To make publishing easier, just give linear and angular velocity
    def pubTwist(self, lin, ang):
    	msg = Twist()
    	msg.linear.x = lin
    	msg.angular.z = ang
    	self.velpub.publish(msg)

    def pointToGrid(self, pt):
    	gridx = int((pt[0]+ (.5*self.res) - self.ox)/self.res)
    	gridy = int((pt[1]+ (.5*self.res) - self.oy)/self.res)
    	r = (gridx, gridy);
    	return r

    def gridToPoint(self, grid):
    	x=(grid[0]*self.res)+self.ox - (.5 * self.res)
    	y=(grid[1]*self.res)+self.oy - (.5 * self.res)
    	r = (x,y);
    	return r

    def getFrontierList(self, data):
        f = []
        for i in range(data):
            x = i%self.w
            y = int(i/self.w)
            if(data[i] < 0):
                #case [x-1,y]
                cur = int( ((x-1) + (y * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x+1,y]
                cur = int( ((x+1) + (y * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x,y-1]
                cur = int( ((x) + ((y-1) * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x,y-1]
                cur = int( ((x) + ((y+1) * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x-1,y-1]
                cur = int( ((x-1) + ((y-1) * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x+1,y-1]
                cur = int( ((x+1) + ((y-1) * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x-1,y+1]
                cur = int( ((x-1) + ((y+1) * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )
                #case [x+1,y+1]
                cur = int( ((x+1) + ((y+1) * self.w)) )
                if(cur > 0 and cur < len(data)):
                    if(data[cur] > 0):
                        f.append((cur%self.w, int(cur/self.w)) )

        return list(set(f))

    def SM():
        # initial state
        if self.state == 0:
            rospy.sleep(0.01)
        # compute a-star state
        elif self.state == 1:
            self.astarpath = self.runAStar()
            if self.astarpath:
                self.waypoints = self.getWaypointTuple(self.astarpath)
                self.state += 1
        # navigate to a star goal state
        elif self.state == 3:
            step = self.executeTraj(self.waypoints)
            if step == 0:
                self.state += 1
        # scan 180 degs
        elif self.state == 4:
            step = self.scan()
            if step == 0:
                self.state += 1
        # scan 180 degs
        elif self.state == 5:
            step = self.scan()
            if step == 0:
                self.state += 1
        # make frontier list
        elif self.state == 6:
            self.frontierList = self.getFrontierList(self.mapdata)
            if(step == 0):
                self.state += 1

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

		# generate possible children
		# dont generate impossible children
	def generateChildren(self):
		global mapData
		global width
		#check if already made children
		data = self.wallExpansion(mapData, self.w, self.expR, 99)
		if not self.children:

			v = self.val + 1
			#case [x-1,y]
			index = int( ((self.coor[0]- 1) + (self.coor[1] * self.w)) )
			if(index > 0):
				if (data[index] != 100):
					temp = ((self.coor[0]-1), self.coor[1]);
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x+1,y]
			index = int((self.coor[0]+ 1) + (self.coor[1] * self.w))
			if(index < len(data)):
				if(data[index] != 100):
					temp = ((self.coor[0]+1), self.coor[1]);
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)

			#case [x,y-1]
			index = int(self.coor[0] + ( (self.coor[1]-1) * self.w))
			if(index > 0):
				if(data[index] != 100):
					temp = (self.coor[0], (self.coor[1]-1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)

			#case [x,y+1]
			index = int(self.coor[0] + ( (self.coor[1]+1) * self.w))
			if(index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0], (self.coor[1]+1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x-1,y-1]
			index = int(self.coor[0]-1 + ( (self.coor[1]-1) * self.w))
			if(index > 0):
				if(data[index] != 100):
					temp = (self.coor[0]-1, (self.coor[1]-1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x-1,y+1]
			index = int(self.coor[0]-1 + ( (self.coor[1]+1) * self.w))
			if(index > 0 and index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0]-1, (self.coor[1]+1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x+1,y-1]
			index = int(self.coor[0]+1 + ( (self.coor[1]-1) * self.w))
			if(index > 0 and index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0]+1, (self.coor[1]-1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			 #case [x+1,y+1]
			index = int(self.coor[0]+1 + ( (self.coor[1]+1) * self.w))
			if(index > 0 and index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0]+1, (self.coor[1]+1));
					baby = Node(v, self, self.start, self.goal, temp)
					self.children.append(baby)

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

def main():
	rospy.init_node('lab5')
    odom_list = tf.TransformListener()
    

if __name__ == '__main__':
    main()
