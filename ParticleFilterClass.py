
"""
Author : Tobe 

ParticleFilterClass.py


"""
import numpy as np 
from RobotClass import Robot 
import random 

PI = np.pi 


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
		normed = np.array(weights)/total_weights#[weights[i]/total_weights for i in range(len(weights))]
		print("Sum of weights = ",total_weights)

		return normed 
		


	def resample(self,number_to_sample = 100):
		""" resamples the particles and generates a new set of particles """ 
		# first, normalise the weights 

		weights = [particle.getWeight() for particle in self.robot_particles]

		ksum = sum(weights)
		epsilon = 1e-100
		if ksum < epsilon: # if the sum is close to zero, return. use the same particles set
			return 

		# compute normalised weights 
		weights_normalised = self.normalise(weights)

		number_of_particles = number_to_sample 
		N = number_to_sample 

		#new_particles_index = []
		new_particles = []

		index = int(random.random() * N) #random.randrange(0,len(weights_normalised)-1)
		max_weight = max(weights_normalised)
		beta = 0.0 
		for i in range(N):
			beta += random.random() * 2 * max_weight # adds a random float uniform(0, 2*max_weight)

			while beta > weights_normalised[index]:  # reduce beta until index where weight is greater than beta
				beta -= weights_normalised[index]
				index  = (index + 1)%N 
			
			#new_particles_index.append(index)
			particle = self.robot_particles[index] 
			turn_noise,forward_noise = particle.getNoise()
			# :: create new copy of the selected particle 
			new_particle_sample = Robot(particle.getPosition(),particle.getOrientation(),self.particle_sensor.clone())
			new_particle_sample.setNoise(turn_noise,forward_noise)

			new_particles.append(new_particle_sample) 

		# new robot particles 
		self.robot_particles = new_particles 

		print ("just Resampled here")

		 
