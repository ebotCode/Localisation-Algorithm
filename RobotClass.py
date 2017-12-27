import numpy as np 
import random 
random.seed(1000)

import math 


import GeometricUtils as geo_util 

PI = np.pi 

class Obstacle: 
	""" class definition of obstacles.
	Obstcles are defined by a list of line segments that represent the obstacles

	""" 
	def __init__(self,boundary_segments):
		"""
		boundary_segments:  represent a list of line segments that define the boundaries of the obstacle. e.g 
			template     :  [line_seg1, line_seg2, ...]
			example 	 : 	[[(0,0),(0,1)],[(0,1),(1,1)],[(1,1),(0,1)],[(0,1),(0,0)]]
							The boundary_segments should be given in anticlockwise order. 
							Internal Ordering Not yet implemented. -------------------------------------
		"""		
		self.boundary_segments = boundary_segments 

	def getBoundaries(self):
		""" returns the list of line segments that define the boundaries of the obstacle """
		return self.boundary_segments 


class WorldMap:
	""" representation of world.
	The world is represented as a set of line segments such that the points are ordered 
	anticlock wisely. (internal ordering is not implemented)
	The world is made of two sets of line segments : 
	1) line segments that define the boundaries of the world 
	2) line segments that define the boundaries of obstacles that are within the world 
		

	""" 
	def __init__(self,bbox,boundary_segments,obstacles = [] ):
		""" args: 
				bbox 			 : bounding rectagle 
						template : [left_top_corner , right_bottom_corner]	
						e.g bbox = [(1,6),(6,1)]

				boundary_segments: list of line segments that define the boundary 
						template : [line_seg1, line_seg2, ...]
						e.g [[(1,1),(4,1)],[(4,1),(4,4)],[(4,4),(1,1)]]

				obstacles        : list of objects of class Obstacle  in the world 
						template : [obstacle1, obstacle2,...]


		"""
		self.boundary_segments = boundary_segments  
		self.obstacles = obstacles   # list of obstacles object 
		self.bounding_box = bbox 

	def getMap(self):
		""" returns all line segments that are present in the world. 
			(including world boundaries and obstacles boundaries) """
		# first assemble all line segments 
		all_line_segments = self.boundary_segments[:]
		if self.obstacles:
			for obstacle in self.obstacles:
				all_line_segments.extend(obstacle.getBoundaries())

		return all_line_segments 

	def isPointLeftOfSegment(self,point_position,line_segment):
		""" returns true if point_position lies to the left of the line_segment, and 
			false otherwise.
			if point lies on line, it also returns false.  

			args: 
				point_position : tuple of        => (x,y) 
				line_segment    : list of tuples => [(x0,y0),(x1,y1)]
		"""

		line_p0 = np.array([[line_segment[0][0]],[line_segment[0][1]]])
		line_p1 = np.array([[line_segment[1][0]],[line_segment[1][1]]])
		point   = np.array([[point_position[0]],[point_position[1]]])

		v1 = line_p1 - line_p0 
		v2 = line_p1 - point 

		answer = geo_util.cross2D(v1,v2)  < 0 
		return answer 

	def isCollidingWithObstacle(self,position):
		""" returns true if position lies inside an obstacle or outside the world 
		 	args: 
		 		position : tuple of (x,y) """  
		does_collide = False 

		# first, check if point is outside the world. 
		for line_seg in self.boundary_segments: 
			if not self.isPointLeftOfSegment(position,line_seg):
				does_collide = True 
				break 

		if does_collide == False :
			# now check if point lies inside any obstacle 
			for obstacle in self.obstacles: 
				is_in_obstacle = True  # point is in obstacle if it is to the left of all obstacle line segment 
				for line_seg in obstacle.getBoundaries():
					if not self.isPointLeftOfSegment(position,line_seg):
						is_in_obstacle = False 
						break

				if is_in_obstacle:
					does_collide = True 
					break  

		return does_collide 



class Sensor:
	def __init__(self,worldmap,sensor_noise = 0.0 ):
		""" 
			args: 
				worldmap  : object of class WorldMap 
				sensor_noise : noise of sensor measurement 

			attributes:
				number_of_sensors : number of sensor ray that model the laser range finder
				start_angle       : the relative angle for the first ray in the sensor 
				rel_ray_directions: list of angles that represent the directions of the rays

		"""
		self.number_of_sensors = 6
		self.start_angle = 0 
		self.worldmap = worldmap  # WorldMap Object
		self.sensor_noise = sensor_noise  
		self.rel_ray_directions = []
		self.computeRayDirections()

	def getNoise(self):
		return self.sensor_noise 

	def setNoise(self,noise):
		self.sensor_noise = noise 

	def computeRayDirections(self):
		""" computes the relative ray directions for the laser range finder sensor """
		angle_step = int(360/self.number_of_sensors)
		self.rel_ray_directions =  [np.pi*i/180 for i in range(self.start_angle,360 + self.start_angle + 1,angle_step)]

	def getDirections(self):
		""" returns the relative ray directions for the laser range finder sensor """
		return self.rel_ray_directions 

	# def senseRayObstruction(self, )


	def measure(self,robot_origin,robot_orientation,sort_measurement = 0 ):
		""" measures the readings from a laser range finder centered about robot_origin.
		 	returns distances to the landmarks (obstaclles ), and ray_segs """ 
		segs = self.worldmap.getMap() 
		full_angle = np.pi*2 
		# compute ray line segments 
		ray_origin = robot_origin 
		rel_ray_directions = self.rel_ray_directions 
		# initialise output arrays 
		ray_segs = []
		intersections = []	
		distances     = []	
		intersecting_segment_indexes = []
		# display ray_directions for visual inspection
		# print ('ray directoins = ',[((rel_ray_directions[i] + robot_orientation)%full_angle)*((180/np.pi) )
		# 							for i in range(len(rel_ray_directions))] )

		if 1: 
			counter = 0 
			# perform ray to line intersection algorithm 
			
			for rel_ray_direction in rel_ray_directions: # go through all the rays 
				min_dist = 100000
				closest_segment_index = 0 # assume the closest segment is at index 0 
				closest_segment_intersection = (0,0)
				for i in range(len(segs)): 	     # go through all the segments 
					counter += 1 
					ray_direction = (rel_ray_direction + robot_orientation)%full_angle			# ray_direction is the sum of relative ray direction and robot orientation mod 2*pi
					bool_ans,value = geo_util.isRayIntersectingLine(ray_origin,ray_direction,segs[i])
					if bool_ans == True: 	 # if it intersects with segment 
						# compute distance to origin 
						dist_to_origin = np.sqrt(np.sum((np.array(ray_origin) - value)**2))
						if dist_to_origin < min_dist:
							min_dist = dist_to_origin
							closest_segment_index = i 
							closest_segment_intersection = tuple(value)


				intersecting_segment_indexes.append(closest_segment_index)
				ray_segs.append([ray_origin,closest_segment_intersection])

				# :: *******************************************
				# :: add noise 
				gaussian_noise = random.gauss(0.0,self.sensor_noise)

				# add noise to coordinate of intersections 
				intersections.append((closest_segment_intersection[0] + gaussian_noise,
									  closest_segment_intersection[1] + gaussian_noise) )
				# add noise to the magnitude of distance 	
				distances.append(min_dist + gaussian_noise ) # add sensor noise 
		else:
			# landmarks = [(1,1),(2,1),(3,1),(4,1),
			# 			 (1,2),(1,3),(1,4),(4,4),
			# 			 (2,4),(3,4)]
			landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]

			distances = [] 
			ray_segs  = []

			gaussian_noise = 0.0 

			for mark in landmarks:
				# dist = np.sqrt(np.sum((np.array(ray_origin) - np.array(mark))**2))
				dist = np.sqrt((ray_origin[0] - mark[0])**2 + (ray_origin[1] - mark[1])**2)
				gaussian_noise = random.gauss(0.0,self.sensor_noise)
					
				distances.append(dist + gaussian_noise)
				ray_segs.append([ray_origin,mark])

		if sort_measurement:
			distances.sort()

		return 	distances,ray_segs 

class ParticleFilter:
	def __init__(self,worldmap,robot,mean_robot,particle_sensor,
					particle_turn_noise,particle_forward_noise,
					number_of_particles ):
		self.worldmap = worldmap 
		self.robot = robot 
		self.robot_particles = []
		self.mean_robot = mean_robot 
		self.world_bbox = worldmap.bounding_box #[(0,4),(4,0)]
		self.start_seed = (0,0)

		self.mean_position = (0,0)  # mean position of all the particles 
		self.mean_orientation = 0 	# mean orientation of all the particles 

		self.particle_sensor = particle_sensor 
		self.particle_turn_noise = 0.0 
		self.particle_forward_noise = 0.0 
		self.number_of_particles = number_of_particles 

		# set particle noise 
		self.setParticleNoise(particle_turn_noise,particle_forward_noise)
		# initialise particles 
		self.initialiseParticles()

		

	def setParticleNoise(self,turn_noise,forward_noise):
		self.particle_turn_noise = turn_noise 
		self.particle_forward_noise = forward_noise 

	def getMeanPositionAndOrientation(self):
		""" returns tuple of mean_position,mean_orientation """
		return self.mean_position,self.mean_orientation 

	def setMeanPositionAndOrientation(self,mean_position,mean_orientation):
		""" set the position and orientation of mean_robot to the mean orientation 
			and position of particle in particle set """
		self.mean_position = mean_position 
		self.mean_orientation   = mean_orientation
		# :: set the mean robot position and orientation to the calculated means 
		self.mean_robot.setPosition(self.mean_position)
		self.mean_robot.setOrientation(self.mean_orientation)
	

	def initialiseParticles(self):
		""" intialises the particles """ 
		# :: create Initial particle positions 
		particle_turn_noise    =  self.particle_turn_noise # 80*PI/180 # 
		particle_forward_noise = self.particle_forward_noise # 10  #0.05 
		# :: 
		world_bbox = self.world_bbox 
		start = self.start_seed 

		x0,y0 = start
		xend,yend = world_bbox[1][0], world_bbox[0][1]


		number_of_particles = self.number_of_particles 
		if 1:
			if 1 : 
				particles = [(random.random()*world_bbox[1][0],random.random()*world_bbox[0][1]) for i in range(number_of_particles)]
				heading_directions = [random.random()*2*PI for i in range(number_of_particles)]
			else:
				particles = [(random.uniform(x0,xend),random.uniform(y0,yend)) for i in range(number_of_particles)]
				heading_directions = [random.uniform(0,2*PI) for i in range(number_of_particles)]
		else: #
			pass 

			# particles = []
			# heading_directions = []	

			# nbins = 20
			# start = self.start_seed 
			# dx = (world_bbox[1][0] - world_bbox[0][0])/nbins 
			# dy = (world_bbox[0][1] - world_bbox[1][1])/nbins 

			# x0 , y0 = (start[0] + dx, start[1] + dy)

			# for row in range(nbins - 1):
			# 	xnew ,ynew = x0, y0 
			# 	for col in range(nbins - 1):
			# 		if not self.worldmap.isCollidingWithObstacle((xnew,ynew)):
			# 			# point coordinate is considered if it is not colliding with any obstacle 
			# 			particles.append((xnew,ynew))
			# 			heading_directions.append(random.random()*2*PI) # 


			# 		xnew,ynew = xnew + dx, y0 
			# 	x0 , y0 = x0, y0 + dy 
			# number_of_particles = len(particles)

		#:: Create Particle Objects 
		self.robot_particles  = []
		for i in range(number_of_particles):
			particle = Robot(particles[i],heading_directions[i],self.particle_sensor)
			particle.setNoise(particle_turn_noise,particle_forward_noise)
			self.robot_particles.append(particle) 


	def computeMeasurementError(self,measurement1,measurement2):
		""" returns the mean error between measurement1 and measurement2 (distances) """ 
		m1 = measurement1[:]
		m2 = measurement2[:]
		m1.sort()
		m2.sort()

		error_value = 0.0 
		error_value2 = 0.0 
		count = len(measurement1) 
		for i in range(len(measurement1)):
			error_value += abs(measurement1[i] - measurement2[i])
			error_value2 += abs(m1[i] - m2[i])

		# compute second measurement error 


		return ( error_value/float(count), error_value2/float(count))

	def computeOrientationError(self,orientation1,orientation2):
		""" returns the absolute error between orientatinon1 and orientation2 """
		return abs(orientation1 - orientation2)


	def error(self):
		""" computes the error in measurment """
		# compute firt type of error (knowledge based error ) wld be removed later
		error_value = 0.0 
		rpos = self.robot.getPosition()
		counter = 0 
		for i in range(len(self.robot_particles)):
			counter += 1 
			ppos = self.robot_particles[i].getPosition()
			dx = (rpos[0] - ppos[0])
			dy = (rpos[1] - ppos[1])

			error_value += np.sqrt(dx*dx + dy*dy)

		error_value = error_value / counter 
		# compute second type of error (measurement based error. )
		# :: get current robot measurements  
		measurement1,_ = self.robot.measure()
		orient1 = self.robot.getOrientation()
		position1     = self.robot.getPosition()
		# :: get the current measurement for mean_robot located at mean and oriented at mean 
		measurement2,_  = self.mean_robot.measure()
		orient2    = self.mean_robot.getOrientation()
		position2    = self.mean_robot.getPosition()
		# :: compare the measurement obtained at the means with the measurement by the robot  
		pos_error1,pos_error2    = self.computeMeasurementError(measurement1,measurement2)
		orient_error = self.computeOrientationError(orient1,orient2)

		# print Results 
		print("*"*50)
		print("==> Error = %.4f"%error_value, "*"*20  )
		print("==> Mean Measurement Error  = %.4f"%pos_error1)
		print("==> Mean Measurement Error2 = %.4f"%pos_error2) # error computed disregarding orientation
		print("==> Mean Orientation Error = %.4f"%(orient_error *(180/PI)))
		print("==> actual position -----> = (%.5f,%.5f)"%position1)		
		print("==> mean   position -----> = (%.5f,%.5f)"%position2)
		print("==> actual  orient  -----> = (%.5f)"%(orient1*180/PI))
		print("==> mean    orient  -----> = (%.5f)"%(orient2*180/PI))
		print("*"*50)		

		
		return (error_value), (pos_error1,orient_error)

	def computeMeanPositionAndOrientation(self):
		""" computes the mean position and orientation of all particles in the particle set
			and sets the value to the pose of the mean_robot  """
		# firt, get the 
		xmean,ymean = 0,0 
		mean_orient = 0.0 
		num_particles = float(len(self.robot_particles))

		for particle in self.robot_particles:
			pos = particle.getPosition()
			orient = particle.getOrientation()
			xmean += pos[0]
			ymean += pos[1] 
			mean_orient += orient 

		mean_position = (xmean/num_particles,ymean/num_particles )
		mean_orientation = (mean_orient/num_particles)

		# set the mean position and orientation 
		self.setMeanPositionAndOrientation(mean_position,mean_orientation)


	def computeNewBoundingBox(self):
		""" returns a new bounding box which is centered around the mean of all particles 
			that are alive at this point, and also returns the lower left corner of the 
			bounding box (start_seed) """
		# first retrieve the mean position of particles 
		mean_position,mean_orientation = self.getMeanPositionAndOrientation()

		width = 20  # width of bounding box 
		# compute bounding box (top left corner, lower right corner)
		bounding_box = [(mean_position[0] - width,mean_position[0] + width),
						(mean_position[0] + width,mean_position[0] - width)]
		# compute lower left corner
		lower_left_corner = (mean_position[0]-width,mean_position[1]-width )	

		return bounding_box, lower_left_corner  

	def localise(self,move_index):
		""" performs single step localisation, and returns the sensor ray segs """
		# ask the robot to pick a direction of motion: 
			# the robot is to scan its environment and move in any direction that is not within
			# 5 units of an obstacle.
		# :: decide motion  
		motions = [self.robot.decideMotion(forward = 5)] # 
		# motions = [(0.1,5),(0.1,5),(0.1,5),(0.1,5),(0.1,5),(0.1,5)]
		move_index = move_index % (len(motions))
		# :: first, make the main robot move 
		self.robot.move(motions[move_index][0], motions[move_index][1])
		# :: then make measurement 
		robot_measurement,sensor_ray_segs = self.robot.measure(sort_measurement = 1) # include sorting for algorithm
		# :: now make all particles make the same movement and compute weights 
		new_particles = []
		N = len(self.robot_particles) 
		for i in range(N):
			particle = self.robot_particles[i]
			particle.move(motions[move_index][0],motions[move_index][1])
			particle.computeWeight(robot_measurement, sort_measurement = 1)	 # sort measurement 		
			new_particles.append(particle)

		self.robot_particles = new_particles 

		# :: resample N new particles 
		self.resample(N)
		# :: compute mean pose    
		self.computeMeanPositionAndOrientation()
		# :: compute Error 
		error_value,mean_errors = self.error() # compute the error		

		# if mean_errors[0] < 10: 
		# 	particle_turn_noise = 2 * PI/180 
		# 	particle_forward_noise = 2 
		# 	self.setParticleNoise(particle_turn_noise,particle_forward_noise)
		# else: 
		# 	particle_turn_noise    =  180*PI/180 # 
		# 	particle_forward_noise = 10  #0.05 	
		# 	self.setParticleNoise(particle_turn_noise,particle_forward_noise)		


		return sensor_ray_segs,mean_errors[0]   # returns the sensor_ray_segs for visual display on GUI

	def getParticles(self):
		return self.robot_particles 

	def normalise(self,weights):
		""" returns a normalised list of weights """
		total_weights = sum(weights)
		normed = [weights[i]/total_weights for i in range(len(weights))]
		print("Sum of weights = ",total_weights)

		return normed 
		


	def resample(self,number_to_sample = 100):
		""" resamples the particles and generates a new set of particles """ 
		# first, normalise the weights 

		weights = [particle.getWeight() for particle in self.robot_particles]

		# compute normalised weights 
		weights_normalised = self.normalise(weights)
		number_of_particles = number_to_sample 
		N = number_to_sample 
		new_particles_index = []
		new_particles = []

		index = int(random.random() * N) #random.randrange(0,len(weights_normalised)-1)
		max_weight = max(weights_normalised)
		beta = 0.0 
		for i in range(N):
			beta += random.random() * 2 * max_weight # adds a random float uniform(0, 2*max_weight)

			while beta > weights_normalised[index]:  # reduce beta until index where weight is greater than beta
				beta -= weights_normalised[index]
				index  = (index + 1)%N 

			new_particles_index.append(index)
			particle = self.robot_particles[index] 
			turn_noise,forward_noise = particle.getNoise()
			# :: create new copy of the selected particle 
			new_particle_sample = Robot(particle.getPosition(),particle.getOrientation(),self.particle_sensor)
			new_particle_sample.setNoise(turn_noise,forward_noise)

			new_particles.append(new_particle_sample) 

		# new robot particles 
		self.robot_particles = new_particles 

		print ("just Resampled here")

		 



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






