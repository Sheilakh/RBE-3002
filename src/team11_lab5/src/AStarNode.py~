class AStarNode(object):

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

		if parent:
			self.path = parent.path[:]
			self.path.append(self.coor)
			self.start = parent.start
			self.goal = parent.goal
		else:
			self.path = [self.coor]
			self.start = start
			self.goal = goal

		self.dist = self.getCost()

	def get_man_dist(self):
		return abs( self.goal[0] - float(self.coor[0]) ) + abs(self.goal[1] - self.coor[1])

	def get_euc_dist(self):
		return math.sqrt( (self.goal[0]-float(self.coor[0]))*(self.goal[0]-float(self.coor[0])) + (self.goal[1] - float(self.coor[1]) )*(self.goal[1] - float(self.coor[1])) )

	def get_diag_dist(self):
		# Best heuristic for 8 grid exansion
		# for extra flair
		D = 1 #cost of 4 dir travel
		D2 = sqrt(2) #cost of 8 dir travel (octile distance, for Chebyshev use D2 =1)
		dx = abs(self.goal[0] - self.coor[0])
		dy = abs(self.goal[1] - self.coor[1])
		return D * (dx + dy) + (D2 - 2 * D) * min(dx,dy)

	def get_cost(self):
		return self.getDiagDist()

	# generate possible children
	# dont generate impossible children
	def generate_children(self, data, map_w):
		global mapData
		global width

		if not self.children:

			v = self.val + 1
			#case [x-1,y]
			index = int( ((self.coor[0]- 1) + (self.coor[1] * map_w)) )
			if(index > 0):
				if (data[index] != 100):
					temp = ((self.coor[0]-1), self.coor[1]);
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x+1,y]
			index = int((self.coor[0]+ 1) + (self.coor[1] * map_w))
			if(index < len(data)):
				if(data[index] != 100):
					temp = ((self.coor[0]+1), self.coor[1]);
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)

			#case [x,y-1]
			index = int(self.coor[0] + ( (self.coor[1]-1) * map_w))
			if(index > 0):
				if(data[index] != 100):
					temp = (self.coor[0], (self.coor[1]-1));
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)

			#case [x,y+1]
			index = int(self.coor[0] + ( (self.coor[1]+1) * map_w))
			if(index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0], (self.coor[1]+1));
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			v = self.val + math.sqrt(2)
			#case [x-1,y-1]
			index = int(self.coor[0]-1 + ( (self.coor[1]-1) * map_w))
			if(index >= 0):
				if(data[index] != 100):
					temp = (self.coor[0]-1, (self.coor[1]-1));
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x-1,y+1]
			index = int(self.coor[0]-1 + ( (self.coor[1]+1) * map_w))
			if(index >= 0 and index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0]-1, (self.coor[1]+1));
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			#case [x+1,y-1]
			index = int(self.coor[0]+1 + ( (self.coor[1]-1) * map_w))
			if(index >= 0 and index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0]+1, (self.coor[1]-1));
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)
			 #case [x+1,y+1]
			index = int(self.coor[0]+1 + ( (self.coor[1]+1) * map_w))
			if(index >= 0 and index < len(data)):
				if(data[index] != 100):
					temp = (self.coor[0]+1, (self.coor[1]+1));
					baby = AStarNode(v, self, self.start, self.goal, temp)
					self.children.append(baby)
