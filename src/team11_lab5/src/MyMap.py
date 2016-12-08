import math

class MyMap(object):
	"""Map helper class"""
	def __init__(self, data= None, w = None, h = None, ox = None, oy = None, res = None, expR = None):

		if data:
			self._data = data
		else:
			self._data = []
		if w:
			self._w = w
		else:
			self._w = 0
		if h:
			self._h = h
		else:
			self._h = 0
		if ox:
			self._ox = ox
		else:
			self._ox = 0
		if oy:
			self._oy = oy
		else:
			self._oy = 0
		if res:
			self._res = res
		else:
			self._res = 0.1
		if expR:
			self._expR = expR
		else:
			self._expR = math.ceil((0.12 * 1.25) / self._res)

	def get_data(self):
		return self._data

	def distance(self, tup1, tup2):
		return math.sqrt((tup2[0]-tup1[0])*(tup2[0]-tup1[0]) + (tup2[1]-tup1[1])*(tup2[1]-tup1[1]))

	def get_height(self):
		return self._h

	def get_width(self):
		return self._w

	def get_resolution(self):
		return self._res

	def update_data(self, data):
		self._data = data.data
		self._w = data.info.width
		self._h = data.info.height
		self._ox = data.info.origin.position.x
		self._oy = data.info.origin.position.y
		self._res = data.info.resolution

	def check_occupancy(self, newdata, path, thresh):
		for node in path:
			index = node[0] + self._w * node[1]
			if(index >= 0 and index < len(newData)):
				if(newdata[index] >= thresh):
					return True
			else:
				print "MyMap.check_occupancy(): error"
				return True
		return False

	# expand the walls of a given mapData
	def expand_walls(self, thresh):
		newData =[]
		for i in self._data:
			newData.append(int(i))
		checked = []
		index = 0
		iterations = 0
		while(iterations < self._expR):
			while(index < len(data)):
				if(index not in checked and self._data[index] > thresh):
					x = index%self._w
					y = int(index/self._w)
					#case [x-1,y]
					cur = int( ((x-1) + (y * self._w)) )
					if(cur > 0 and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
					cur = int( ((x+1) + (y * self._w)) )
					if(cur < len(data) and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
					cur = int( (x + ((y-1) * self._w)) )
					if(cur > 0 and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
					cur = int( (x + ((y+1) * self._w)) )
					if(cur < len(data) and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)

					cur = int( ((x-1) + ((y-1) * self._w)) )
					if(cur > 0 and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
					cur = int( ((x+1) + ((y-1) * self._w)) )
					if(cur > 0 and cur < len(data) and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
					cur = int( ((x-1) + ((y+1) * self._w)) )
					if(cur > 0 and cur < len(data) and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
					cur = int( ((x+1) + ((y+1) * self._w)) )
					if(cur < len(data) and newData[cur] < self._data[index]):
						newData[cur] = self._data[index]
						checked.append(cur)
				index += 1
			index = 0
			iterations += 1
			self._data = newData
			checked = []
		return self._data

	def point_to_grid(self, pt):
		gridx = int((pt[0]+ (.5*self._res) - self._ox)/self._res)
		gridy = int((pt[1]+ (.5*self._res) - self._oy)/self._res)
		return (gridx, gridy);

	def grid_to_point(self, grid):
		x=(grid[0]*self._res)+self._ox - (.5 * self._res)
		y=(grid[1]*self._res)+self._oy - (.5 * self._res)
		return (x,y);

	def get_frontier_list(self, data):
		f = []
		for i in range(data):
			x = i%self._w
			y = int(i/self._w)
			if(data[i] < 0):
				#case [x-1,y]
				cur = int( ((x-1) + (y * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self.w, int(cur/self.w)) )
				#case [x+1,y]
				cur = int( ((x+1) + (y * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self.w, int(cur/self.w)) )
				#case [x,y-1]
				cur = int( ((x) + ((y-1) * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self.w, int(cur/self.w)) )
				#case [x,y-1]
				cur = int( ((x) + ((y+1) * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self._w, int(cur/self._w)) )
				#case [x-1,y-1]
				cur = int( ((x-1) + ((y-1) * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self._w, int(cur/self._w)) )
				#case [x+1,y-1]
				cur = int( ((x+1) + ((y-1) * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self._w, int(cur/self._w)) )
				#case [x-1,y+1]
				cur = int( ((x-1) + ((y+1) * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self._w, int(cur/self._w)) )
				#case [x+1,y+1]
				cur = int( ((x+1) + ((y+1) * self._w)) )
				if(cur >= 0 and cur < len(data)):
					if(data[cur] >= 0):
						f.append((cur%self._w, int(cur/self._w)) )

		return list(set(f))


	# Return a list of tuples corresponding to
	# grid cell coordinates of waypoints
	def get_waypoint_tuple(self, path):
		poselist = []
		numsq = 0
		direc = 0
		prev = -1
		difx = path[1][0] - path[0][0]
		dify = path[1][1] - path[0][1]
		poselist.append(path[0])
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

		if(len(path)-1 == 2):
			return tlist

		for i in xrange(2,len(path)-1):
			difx = path[i][0] - path[i-1][0]
			dify = path[i][1] - path[i-1][1]
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
				poselist.append(path[i-1])
			prev = direc
		poselist.append(path[-1])
		return poselist
