import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt
import argparse

class Obstacle():
	def __init__(self, width = 10, height = 10, r = 1, c = 1, threshold=0.01, 
			thetaStep = 30, actions=None, wheelLength = 1, 
			wheelRadius = 1):
		self.threshold = threshold                                                # Resolution
		self.W = int(width/threshold) +1
		self.H = int(height/threshold) +1
		self.r = r
		self.c = c
		self.thetaStep = thetaStep
		self.explored = np.zeros([self.H, self.W, 4])
		self.actionIndexMatrix = np.zeros([self.H, self.W])
		self.plotData_X = []
		self.plotData_Y = []
		self.plotData_A = []
		self.plotData_U = []
		self.plotData_V = []
		self.whcihAction = []
		plt.ion()
		self.fig, self.ax = plt.subplots()
		plt.axis('square')
		self.plotSpace()
		self.actions = actions
		self.wheelRadius = wheelRadius
		self.wheelLength = wheelLength

	
	def plotSpace(self):
		centX, centY, radii = 2,2,1
		circle_1_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
		circle_1_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
		centX, centY, radii = 2,8,1
		circle_2_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
		circle_2_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
		quad_1_x, quad_1_y = [0.25, 1.75, 1.75, 0.25, 0.25],[ 4.25,  4.25,  5.75,  5.75,  4.25]
		quad_2_x, quad_2_y = [3.75, 6.25, 6.25, 3.75, 3.75],[ 4.25,  4.25, 5.75, 5.75,  4.25]
		quad_3_x, quad_3_y = [7.25, 8.75, 8.75, 7.25, 7.25],[  2,   2,  4,  4,  2]
		self.ax.plot(circle_1_X, circle_1_Y)
		self.ax.plot(circle_2_X, circle_2_Y)
		self.ax.plot(quad_1_x, quad_1_y)
		self.ax.plot(quad_2_x, quad_2_y)
		self.ax.plot(quad_3_x, quad_3_y)
		self.ax.set_xlim(0, 10)
		self.ax.set_ylim(0, 10)
		pass

	def checkObstcaleSpace(self):
		xx = np.arange(0,10,0.05)
		yy = np.arange(0,10,0.05)
		x_ = []
		y_ = []
		for x in xx:
			for y in yy:
				if self.ObsCheck(x,y):
					x_.append(x)
					y_.append(y)
		plt.scatter(x_, y_, s=0.1)
		plt.show()
		pass

	def ObsCheck(self, i, j):
		if self.checkBoundary(i,j):
			return False
		elif self.checkInCircle(i, j, (2,2), 1):
			return False
		elif self.checkInCircle(i, j, (2,8), 1):
			return False
		elif self.checkInQuad(i, j, 0.25, 1.75, 5.75, 4.25):
			return False
		elif self.checkInQuad(i, j, 3.75, 6.25, 5.75, 4.25):
			return False
		elif self.checkInQuad(i, j, 7.25, 8.75, 4, 2):
			return False
		else:
			return True

	def checkInQuad(self, i, j, left, right, top, bottom):
		l_ = left - self.r - self.c
		r_ = right + self.r + self.c
		t_ = top + self.r + self.c
		b_ = bottom - self.r - self.c
		if (i<r_ and i>l_ and j<t_ and j>b_):
			return True
		else:
			return False

	def checkInCircle(self, i, j, center, radius):
		center_x, center_y = center[0], center[1]
		if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.r + self.c) ** 2:
			return True
		else:
			return False

	def checkBoundary(self,i,j):
		# print(i,j)
		if (i < (10 - self.r - self.c) and  
			i > (0 + self.r + self.c) and 
			j < (10 - self.r - self.c) and  
			j > (0 +self.r + self.c)):
			return False
		return True

	def getMatrixIndices(self, node):
		x,y,a = node[1], node[2], node[3]
		shiftx, shifty = 0,0
		x += shiftx
		y = abs(shifty + y)
		i = int(round(y/self.threshold))
		j = int(round(x/self.threshold))
		k = int(round(a/self.thetaStep))
		return i,j,k

	def checkVisited(self, node):
		i,j,k = self.getMatrixIndices(node)
		if self.explored[i, j, 3] != 0:
			return True 
		else:
			return False 

	def findVisited(self, node):
		i,j,k = self.getMatrixIndices(node)
		return self.explored[i, j, :], self.actionIndexMatrix[i,j]

	def addVisited(self, node, parentNode, actionIndex):
		i,j,k = self.getMatrixIndices(node)
		self.plotData_X.append(parentNode[1])
		self.plotData_Y.append(parentNode[2])
		self.plotData_A.append(parentNode[3])
		self.whcihAction.append(actionIndex)
		self.explored[i, j, :] = np.array(parentNode)
		self.actionIndexMatrix[i,j] = actionIndex
		return

	def plotPath(self, path, trackIndex):
		print(len(trackIndex), len(path))
		for i in range(len(path)):
			Xi = path[i][1]
			Yi = path[i][2]
			Thetai = path[i][3]
			actionIndex = int(trackIndex[i])
			UL, UR = self.actions[actionIndex][0], self.actions[actionIndex][1]
			self.plotCurve(Xi, Yi, Thetai, UL, UR, color="red", lw=1.2)
			plt.pause(0.00001)
		plt.ioff()
		plt.show(block=False)
		pass

	def explorationPlot(self):
		for i in range(len(self.plotData_X)):
			Xi = self.plotData_X[i]
			Yi = self.plotData_Y[i]
			Thetai = self.plotData_A[i]
			actionIndex = self.whcihAction[i]
			UL, UR = self.actions[actionIndex][0], self.actions[actionIndex][1]
			self.plotCurve(Xi, Yi, Thetai, UL, UR)
			if i%100==0:
				plt.pause(0.000001)
		pass

	def plotCurve(self, Xi, Yi, Thetai, UL, UR,color="blue",lw=0.5):
		r = self.wheelRadius
		L = self.wheelLength
		t = 0
		dt = 0.1
		Xn = Xi
		Yn = Yi
		Thetan = 3.14 * Thetai / 180
		x_s, x_n, y_s, y_n = [],[],[],[] 
		while t<1:
			t = t + dt
			Xs = Xn
			Ys = Yn
			Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
			Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
			Thetan += (r / L) * (UR - UL) * dt
			x_s.append(Xs)
			x_n.append(Xn)
			y_s.append(Ys)
			y_n.append(Yn)
		self.ax.plot([x_s, x_n], [y_s, y_n], color=color, linewidth=lw)

class pathFinder():
	def __init__(self, initial, goal, thetaStep = 30, stepSize = 1, goalThreshold = 0.1,
		width = 10, height = 10, threshold = 0.5, r = 0.1, c = 0.1, wheelLength = 0.038, 
		Ur=18,Ul=15, wheelRadius=2, dt=0.1, dtheta=0, weight=1, showExploration=0, showPath=1):
		self.initial = initial
		self.goal = goal
		# node = [ x , y , angle , cost ]
		self.nodeData = []
		self.weight = weight
		self.Data = []
		self.allData = []
		self.thetaStep = thetaStep
		self.dt=dt
		self.dtheta=dtheta
		self.wheelRadius=wheelRadius
		self.wheelLength=wheelLength
		self.Ur=Ur
		self.Ul=Ul
		self.stepSize = stepSize
		self.goalThreshold = goalThreshold
		self.path = []
		self.trackIndex = []
		self.goalReach = False
		self.actions = [[	   0 , self.Ur],
						[self.Ul , 		 0],
						[	   0 , self.Ul],
						[self.Ur , 		 0],
						[self.Ul , self.Ur],
						[self.Ur , self.Ul],
						[self.Ur , self.Ur],
						[self.Ul , self.Ul]]
		self.actionSet = []
		self.obstacle = Obstacle(width, height, r = r, c = c, threshold=threshold, 
			actions=self.actions, wheelLength = self.wheelLength, 
			wheelRadius = self.wheelRadius)
		self.showExploration = showExploration
		self.showPath = showPath

	def setActions(self, presentNode):
		self.actionSet = []
		index = 0
		for action in self.actions:
			t = 0
			dt = 0.1
			x, y, angle = presentNode[1], presentNode[2], presentNode[3]
			angle = 3.14*angle/180.0 
			costToCome = 0
			for i in range(10):
				t = t+dt
				xnew = 0.5*(self.wheelRadius)*(action[0]+action[1])*math.cos(angle)*dt
				ynew = 0.5*(self.wheelRadius)*(action[0]+action[1])*math.sin(angle)*dt
				x += xnew       
				y += ynew      
				angle += (self.wheelRadius/self.wheelLength)*(action[1]-action[0])*dt              
				costToCome += math.sqrt(xnew ** 2 + ynew ** 2)
			angle = 180 * (angle) / 3.14
			self.actionSet.append([x, y, angle, costToCome, index])
			index += 1

	def initialCheck(self):
		if not self.obstacle.ObsCheck(self.goal[0], self.goal[1]):
			print("Goal in obstacle field")
			return False
		elif not self.obstacle.ObsCheck(self.initial[0], self.initial[1]):
			print("Initial position in obstacle field")
			return False
		else:
			cost = math.sqrt((self.initial[0] - self.goal[0])**2 + (self.initial[1] - self.goal[1])**2)
			heappush(self.Data, [cost, self.initial[0], self.initial[1], self.initial[2], 0])
			self.nodeData.append([self.initial[0], self.initial[1], self.initial[2], 0])
			return True

	def heuristics(self, current): 
		h = self.weight * math.sqrt((current[1] - self.goal[0])**2 + (current[2] - self.goal[1])**2)
        
		return h

	def goalReached(self, current):  
		x, y = current[1], current[2]
		if (x - self.goal[0])**2 + (y - self.goal[1])**2 <= (self.goalThreshold)**2:
			return True
		else:
			return False

	def trackBack(self, presentNode):
		track = []
		trackIndex = []
		currentNode = presentNode[:4]
		track.append(currentNode)
		trackIndex.append(0)
		while currentNode[1:] != self.initial:
		
			l, ind = self.obstacle.findVisited(currentNode)
			currentNode = list(l)
			track.append(currentNode)
			trackIndex.append(ind)
		print("-------------------")
		print("Backtracking")
		track.reverse()
		trackIndex.reverse()
		return track, trackIndex

	def findPath(self):
		counter = 0
		if self.initialCheck():
			while len(self.Data)>0:
				counter+=1
				presentNode = heappop(self.Data)
				previousCost, previousCostToCome = presentNode[0], presentNode[4]
				if self.goalReached(presentNode):
					self.goalReach = True
					print(" Goal Reached ")
					self.path, self.trackIndex = self.trackBack(presentNode)
					if self.showExploration:
						self.obstacle.explorationPlot()
					if self.showPath:
						self.obstacle.plotPath(self.path, self.trackIndex)
					return
				self.setActions(presentNode)
				for action in self.actionSet:
					newNodeX = action[0]
					newNodeY = action[1]
					newNodeA = action[2]
					newNode = [0, newNodeX, newNodeY, newNodeA, 0]
					newCostToCome = previousCostToCome + action[3]
					newNode[4] = newCostToCome
					costToGo = self.heuristics(newNode)
					if self.obstacle.ObsCheck(newNodeX, newNodeY):
						if not self.obstacle.checkVisited(newNode):
							presentNode[0] = newCostToCome
							self.obstacle.addVisited(newNode, presentNode[:4], action[4])
							newNode[0] = newCostToCome + costToGo
							heappush(self.Data, newNode)
						else: 
							previousVisited,_ = self.obstacle.findVisited(newNode)
							previousCost = previousVisited[0]
							if previousCost > newCostToCome:
								presentNode[0] = newCostToCome
								self.obstacle.addVisited(newNode, presentNode[:4], action[4])
		print("Could not reach goal..") 
		return

if __name__ == '__main__':
	Parser = argparse.ArgumentParser()
	Parser.add_argument('--Start', default="[1, 1, 30]", help='Inital location')
	Parser.add_argument('--End', default="[9, 9, 0]", help='Goal location')
	Parser.add_argument('--RobotRadius', default=0.177, help='Robot radius')
	Parser.add_argument('--Clearance', default=0.1, help='Clearance')
	Parser.add_argument('--ShowExploration', default=0, help='1 for exploration animation else 0')
	Parser.add_argument('--ShowPath', default=1, help='1 to show explored path else 0')
	Parser.add_argument('--thetaStep', default=30, help='Possibilities of action for angle')
	Parser.add_argument('--StepSize', default=2, help='Step size')
	Parser.add_argument('--Threshold', default=0.01, help='Threshold value for appriximation')
	Parser.add_argument('--GoalThreshold', default=0.1, help='Threshold for goal')
	Parser.add_argument('--WheelRadius', default=0.038, help='Radius of the robot wheel in meters')
	Parser.add_argument('--WheelLength', default=0.320, help='wheelbase')
	Parser.add_argument('--RPM', default="[15,18]", help='RPM values')
	Parser.add_argument('--Weight', default=1.3, help='Weight for cost to go')
	Args = Parser.parse_args()

	start = Args.Start
	end = Args.End
	r = float(Args.RobotRadius)
	c = float(Args.Clearance)
	StepSize = int(Args.StepSize)
	Threshold = float(Args.Threshold)
	GoalThreshold = float(Args.GoalThreshold)

	initial = [float(i) for i in start[1:-1].split(',')]
	goal = [float(i) for i in end[1:-1].split(',')] 
	print(initial)
	print(goal)
	wheelLength = float(Args.WheelLength) 
	
	rpm = [float(i) for i in Args.RPM[1:-1].split(',')]
	Ur, Ul = rpm[0], rpm[1]

	wheelRadius = float(Args.   WheelRadius)
	weight = float(Args.Weight)

	solver = pathFinder(initial, goal, stepSize=StepSize,
		goalThreshold = GoalThreshold, width = 10, height = 10, threshold = Threshold,
		r=r, c=c, wheelLength = wheelLength, Ur = Ur, Ul = Ul, wheelRadius = wheelRadius,
		weight = weight, showExploration=int(Args.ShowExploration), showPath=int(Args.ShowPath))
	solver.findPath()
	print(solver.trackIndex)
	l = wheelLength
	r = wheelRadius
	for idx in solver.trackIndex:
		ul, ur = solver.actions[int(idx)] 
		vx = r*0.5*(ur+ul)
		rz = r*(ur-ul)/l
		print(ul,ur,vx,rz)


