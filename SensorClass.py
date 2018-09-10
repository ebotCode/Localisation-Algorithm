"""
Author : Tobe 

SensorClass.py
"""
import GeometricUtils as geo_util 
import numpy as np 

import random 

PI = np.pi 


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
		self.number_of_sensors = 8
		self.start_angle = 0 
		self.worldmap = worldmap  # WorldMap Object
		self.sensor_noise = sensor_noise  
		self.rel_ray_directions = []
		self.computeRayDirections()

	def clone(self):
		cloned_sensor = Sensor(self.worldmap,self.getNoise())
		return cloned_sensor 

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

