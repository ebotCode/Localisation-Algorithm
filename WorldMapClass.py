"""
Author : Tobe 

WorldMapClass.py

"""

import numpy as np 

import GeometricUtils as geo_util 

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

