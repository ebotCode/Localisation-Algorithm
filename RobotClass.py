import numpy as np 
import random 
import Settings as settings 
import math 


PI = np.pi 

#random.seed(settings.SEED_VALUE)

class Robot:
	def __init__(self,position,orientation,sensor):
		self.position  = (0,0)
		self.orientation = 0 # heading direction in radians 

		self.forward_noise = 0.0 
		self.turn_noise  = 0.0 	
		self.sensor = sensor  

		self.weight = 1 
		
		self.reverse_count = 0 
		self.setPosition(position)
		self.setOrientation(orientation)


	def getPosition(self):
		return self.position 

	def getOrientation(self):
		return self.orientation 

	def setPosition(self,position):
		self.position = position 

	def setOrientation(self,orientation):
		""" sets the orientation of the robot. 
			orientation is the absolute direction (0,2*pi) measured 
			from the positive x-direction (anti-clockwise)
		"""
		if (0 <= orientation) and (orientation <= np.pi*2):
			self.orientation = orientation 
		else:
			raise ValueError("invalid value for orientation ")		

	def __str__(self):
		pos = self.getPosition()
		return '[x = %.4f, y = %.4f, orientation = %.4f]'%(pos[0],pos[1],self.getOrientation())


	def setNoise(self,turn_noise,forward_noise):
		self.turn_noise = turn_noise 
		self.forward_noise = forward_noise 

	def getNoise(self):
		return (self.turn_noise,self.forward_noise)

	def gaussian(self,mu, sigma,x):
		# calculates the gaussian 
		return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * PI * (sigma ** 2))


	def computeWeight(self,measurement,sort_measurement = 0):
		""" this computes the weight for the robot, given the main_robot_measurement. 
			The weight represents how likely the measurement was obtained by this 
			robot. 
			args:
				measurement: is a list of sensor distances to obstacles 
				sort_measurement: boolean that tells wether the results form the sensor should be 
								  sorted or not. default is 0 
										
		"""
		prob = 1.0 
		distances,_  = self.measure(sort_measurement = sort_measurement) # include sorting 
													  # wen computing weight for the particle 
		for i in range(len(distances)): 
			prob *= self.gaussian(distances[i],self.sensor.getNoise(),measurement[i])
		
		self.weight = prob 
		# print("myweight = ",self.weight)


	def getWeight(self):
		return self.weight 


	def measure(self,sort_measurement = 0 ):
		return self.sensor.measure(self.getPosition(),self.getOrientation(),sort_measurement)

	def decideMotion(self,forward):
		""" returns the (turn,distance) the robot wants to move. 
			it chooses a direction that is at least forward units away from an obstacle """

		turn = 0.1 # assume the current direction is preferred. 
		directions = self.sensor.getDirections()
		distances,_ = self.measure(sort_measurement = 0 ) # dont sort measurement so that it can match directions
		# randomly pick any direction that has a distance > 5 unit 
		number_of_measurement = len(directions)
		for index in range(number_of_measurement):
			index_i = index #int(random.random()*number_of_measurement)
			if distances[index_i] > forward:
				turn = directions[index_i]
				break 

		return (turn,forward)



	def move(self,relative_direction,distance):
		""" performs robot motion. 
			relative_direction:  is a float value that specifies the 
								relative orientation for the motion. if relative_direction is positive, it means the 
								robot is to turn anti-clockwise by an amount equal to the abs(relative_direction). if 
								relative_direction is negative, the robot is to turn clockwise relative to the orientation
			distance : this is the distance the robot is to travel in its new direction 

		"""

		x,y = self.getPosition()
		full_angle = np.pi * 2 
		current_orientation = self.getOrientation()

		# compute new orientation  
		orient_noise= random.gauss(0.0,self.turn_noise)
		new_orientation = ( current_orientation + relative_direction + orient_noise) % full_angle  
		if new_orientation < 0: 
			new_orientation = full_angle + new_orientation 

		# compute new robot position 
		dnoise = random.gauss(0.0,self.forward_noise)
		dist = distance + dnoise 

		new_x = x + dist * math.cos(new_orientation)
		new_y = y + dist * math.sin(new_orientation)

		# new_x %= 100.0
		# new_y %= 100.0

		# print('myactual Computed Pos  ====================> ',(new_x,new_y))
		# print('my x computation was with ===> ',(x,dist,math.cos(new_orientation)))
		# print('my y computation was with ===> ',(y,dist,math.sin(new_orientation)))

		# check if this new_x,new_y is not coliding in any way 
		# if self.sensor.worldmap.isCollidingWithObstacle((new_x,new_y)) and self.reverse_count <= 3:
		# 	# turn 180 deg around and move 
		# 	self.reverse_count += 1 
		# 	self.move(PI,distance) # pick a random direction and move 

		# 	print("Just turned around************************************************")
		# else:
		# 	self.reverse_count = 0 
		# 	# set the new coordinates 
		# 	self.setPosition((new_x,new_y))
		# 	self.setOrientation(new_orientation)
 

		self.setPosition((new_x,new_y))
		self.setOrientation(new_orientation)	
		# self.setPosition((xa,ya))
		# self.setOrientation(oa)	







