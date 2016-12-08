import rospy,tf
import AStarNode
import MyMap
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion

class Robot(object):

	def __init__(self, vel_pub, path_pub, wall_pub, robot_map, v = False):
		self._x = 0
		self._y = 0
		self._z = 0

		self._tx = 0
		self._ty = 0
		self._tz = 0

		self._traj_step = 2 # Skip first pose, robot is already there
		self._tempgoalset = False

		self._gx = -1
		self._gy = -1
		self._gz = -360
		self._state = 0

		self._astarpath = []
		self._waypoints = []

		self._vel_pub = vel_pub
		self._path_pub = path_pub
		self._wall_pub = wall_pub
		self._map = robot_map # map data struct
		self._odom_list = tf.TransformListener()

		# Verbose flag, if you want print statements
		self._v = v

	# Goal callback function
	# set the global goal variables
	# and execute the trajectory using A-star search
	# goal is PoseStamped
	def set_goal(self, goal):
		if self._v:
			print 'Robot.set_goal()'
			print goal.pose
			print self._map.point_to_grid((goal.pose.position.x, goal.pose.position.y))
			print ""
		self._gx = goal.pose.position.x
		self._gy = goal.pose.position.y
		quat = (
			goal.pose.orientation.x,
			goal.pose.orientation.y,
			goal.pose.orientation.z,
			goal.pose.orientation.w)
		self._z = euler_from_quaternion(quat)[2]
		self._state = 1
		return 1

	def update_map(self, new_data):
		self._map.update_data(new_data)

	def _update_coordinates(self, pose):
		self._x = pose.position.x
		self._y = pose.position.y
		# Convert goal orientation from quaternion to a yaw angle
		quat = (
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w)

		self._z = euler_from_quaternion(quat)[2]


	# Run A star search
	def _run_a_star(self, start_tup, goal_tup):
		sp = self._map.point_to_grid((start_tup[0], start_tup[1]))
		gp = self._map.point_to_grid((goal_tup[0], goal_tup[1]))
		start = AStarNode(0, None, sp, gp, sp)
		path = []
		visited = []
		pqueue = PriorityQueue()
		count = 0
		pqueue.put((0,start))
		if self._v:
			   print"Starting A-star search"

		while(not path and pqueue.qsize()):
			closest = pqueue.get()[2]
			if self._v:
				print "got node", str(closest.coor)
			closest.generate_children(self._map.get_data(),self._map.get_width())
			visited.append(closest.coor)
			for c in closest.children:
				if c.coor not in visited:
					if not c.get_man_dist():
						path = c.path
						break
					pqueue.put((c.dist,c))
		if not path:
			if self._v:
					  print "no path"
		else:
			if self._v:
				print "found path"
				for p in path:
					print p
				print
		return path

	# This function accepts a speed and a distance for the robot to move in a straight line
	# Returns whether or not the robot has completed that distance
	def _drive_straight(self, speed, goaldist):
		# Record initial position
		if(not self._tempgoalset):
			self._tx = self._x + goaldist*math.cos(self._z)
			self._ty = self._y + goaldist*math.sin(self._z)
			self._tempgoalset = True
			
		# Distance between current pose and drive goal
		dist = self._map.distance((self._x, self._y),(self._tx,self._ty))
		atTarget = dist < 0.02

		# Drive (twist(speed,0)) until distance has passed, based on current and initial positions
		if not rospy.is_shutdown():
			if atTarget:
				self._pub_twist(0,0)
				self._tempgoalset = False
				if self._v:
					print "_drive_straight done"
				return 0
			else:
				if(dist/goaldist < 0.5):
					speed = (dist/goaldist)*speed
				else:
					speed = ((goaldist-dist)/goaldist)*speed
				if(speed < 0.1):
					speed = 0.1
				self._pub_twist(speed,0)
				return dist

		return -1
	#Accepts a relative angle and makes the robot rotate by that much. (-180 to 180)
	def _rotate(self, speed, theta):

		if theta < -180 or theta > 180:
			print "Keep angle within -180 to 180"

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
		if(not self._tempgoalset):
			self._tz = self._z+theta
			self._tempgoalset = True
		traveled =  self._tz - self._z

		# Account for wrap around
		if traveled > 180:
			traveled = -360 + traveled
		elif traveled < -180:
			traveled = 360 + traveled

		atTarget = abs(traveled) >= abs(theta)
		# Drive (twist(speed,0)) until distance has passed, based on current and initial positions
		if not rospy.is_shutdown():
			if atTarget:
				self._pub_twist(0,0)
				self._tempgoalset = False
				if self._v:
					print "_rotate done"
				return 0
			else:
				if(traveled/theta < 0.5):
					speed = (traveled/theta)*speed
				else:
					speed = ((theta-traveled)/theta)*speed
				if(speed < 0.1):
					speed = 0.1
				self._pub_twist(0,speed)
				return traveled

		return -1

	# Drive directly to a pose
	# goal is PoseStamped
	def _nav_to_pose(self, goal_tup):

		# Calculate the angle and distance to the goal
		deltaX = goal_tup[0] - self._x
		deltaY = goal_tup[1] - self._y
		theta = math.degrees(math.atan2(deltaY, deltaX)) # Global frame
		dist = math.sqrt(deltaX**2 + deltaY**2)

		# Rotate until facing the goal
		if(self._traj_step%2 == 0):
			d = self._rotate(0.5. theta)
			if (d != 0):
				return (1, d)
			else:
				self._traj_step += 1
				return (1,0)
		else:
			d = self._drive_straight(0.3, dist)
			if(d != 0):
				return (2,d)
			else:
				self._traj_step += 1
				return (2,0)

	def _execute_trajectory(self, plist):
		i = int(self._traj_step/2)
		termtest = self._nav_to_pose(self._map.grid_to_point(plist[i]))
		if (termtest == (2,0)):
			if (i == len(plist)-1):
				self._traj_step = 2 # Reset step count
				if self._v:
					print "completed trajectory"
				return 0

		return termtest[1]

	# Just rotate 180 degrees in place, probably always run twice
	def _scan(self):
		return self._rotate(180,0.5)

	# Publish the A* path
	def _pub_astar_path(self):
		path = self._astarpath
		if self._v:
			   print "Publishing path."
		cells = GridCells()
		cells.header.frame_id = 'map'
		cells.cell_width = self._map.get_resolution()
		cells.cell_height = self._map.get_resolution()
		for i in range(len(path)):
			point=Point()
			pt = self._map.grid_to_point((path[i][0], path[i][1]) )
			point.x = pt[0]
			point.y = pt[1]
			point.z = 0
			cells.cells.append(point)

		self._path_pub.publish(cells)

	# publishes map to rviz using gridcells type
	def _pub_wall_cells(self):
		# self.res and offset of the map
		cells = GridCells()
		cells.header.frame_id = 'map'
		cells.cell_width = self._map.get_resolution()
		cells.cell_height = self._map.get_resolution()
		k=0
		grid = self._map.get_data()
		for i in range(1,self._map.get_height()+1): #height should be set to height of grid
			for j in range(1,self._map.get_width()+1): #width should be set to width of grid
				if (grid[k] == 100):
					point=Point()
					pt = self._map.grid_to_point((j,i))
					point.x= pt[0]
					point.y= pt[1]
					point.z=0
					cells.cells.append(point)
				k+=1
		self._wall_pub.publish(cells)

	# To make publishing easier, just give linear and angular velocity
	def _pub_twist(self, lin, ang):
		msg = Twist()
		msg.linear.x = lin
		msg.angular.z = ang
		self._vel_pub.publish(msg)

	# Timer callback function
	# publish grid cells
	def wall_pub_timer_cb(self, timerevent):
		if self._v:
			print "publishing walls"
		self._pub_wall_cells()

	# Timer callback function
	# read odometry data
	def odom_timer_cb(self, timerevent):
		self._odom_list.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(1, 0))
		c_p, c_q = self._odom_list.lookupTransform("/map", "/base_footprint", rospy.Time(0))
		_current_pose = Pose()
		_current_pose.position.x = c_p[0]
		_current_pose.position.y = c_p[1]
		_current_pose.position.y = c_p[2]
		_current_pose.orientation.x = c_q[0]
		_current_pose.orientation.y = c_q[1]
		_current_pose.orientation.z = c_q[2]
		_current_pose.orientation.w = c_q[3]
		self._update_coordinates(_current_pose)

	def path_pub_timer_cb(self, timerevent):
		if(self._astarpath):
			if self._v:
				print "publishing path"
			self._pub_astar_path()


	def execute(self):
		# Initial state
		if self._state == 0:
			#if self._v:
			#	print "state 0"
			rospy.sleep(0.2)
		# Compute a-star state
		elif self._state == 1:
			if self._v:
				print "State 1: Running A*."
			self._astarpath = self._run_a_star((self._x,self._y),(self._gx,self._gy))
			if self._astarpath:
				self._waypoints = self._map.get_waypoint_tuple(self._astarpath)
				self._state += 1
		# Navigate to a star goal state
		elif self._state == 2:
			if self._v:
				print "State 2: Executing trajectory."
			step = self._execute_trajectory(self._waypoints)
			if step == 0:
				self._state += 1
		# Scan 360 degs (rotate 180 twice)
		elif self._state == 3:
			if self._v:
				print "State 3: Scanning."
			if self._scan() == 0:
				self._state += 1
		# Make frontier list
		elif self._state == 4:
			if self._v:
				print "State 4: Make frontier list.
			self.frontierList = self.map.get_frontier_list(self._map.get_data())
			if(step == 0):
				self._state += 1
		# Shouldn't be reachable
		else:
			if v:
				print "Error, invalid state."
			return -1
