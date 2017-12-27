

""" 

"""
class PointObject: 
	point_ids = 0 
	def __init__(self, parent = None , point= (0,0)):
		""" parent = None, point = (0,0) """ 
		self.parent = parent 
		self.point = []
		self.setPoints(point)
		PointObject.point_ids += 1

	def setPoints(self,point):
		if len(point) > 2: 
			raise ValueError ("Invalid argument for point in PointObject ")
		self.point = point 

	def getPoints(self):
		return self.point


class LineObject:
	""" Line Object """ 
	line_ids = 0
	def __init__(self,parent = None , points = [(0,0),(0,1)]):
		""" parent = None, points = list of tuple coordinates """ 
		self.parent = parent 
		self.points = []				# stores a list of point objects 
		self.id = LineObject.line_ids 
		self.constructLine(points)

		LineObject.line_ids += 1

	def constructLine(self,points):
		if len(points) < 2: 
			raise ValueError("Invalid input for points in LineObject" )

		self.points = []
		for item in points: 
			self.points.append(PointObject(self,item))


	def setPoints(self,points):
		self.constructLine(points)

	def getPoints(self):
		""" returns the points """ 
		return [p.getPoints() for p in self.points ]


class VisualObject: 
	object_ids = 0 
	def __init__(self,sequence_points = [],color = "black"):
		""" sequence object is a list of points in sequence that define the object to be drawn """
		self.boundaries = []
		self.color      = color 
		self.setSequence(sequence_points)

	def drawObject(self,guiobj):
		""" draws the visual object on the canvas referenced by guiobj """ 
		line_segments = self.getPoints()
		object_color  = self.getColor()
		for line in line_segments:
		    p1 = line[0]
		    p2 = line[1]
		    guiobj.displayLine(p1,p2,fill = object_color) 



	def setSequence(self,sequence_points):
		""" sets the sequence_lines that define the boundaries of the object. 
			Note: sequence_points must contain at lest one line e.g [(0,0),(1,1)] """
		self.constructObject(sequence_points)


	def constructObject(self,sequence_points):
		""" constructs the object as a list of line segments that define the object """ 
		self.boundaries = []
		if sequence_points == []: 
			self.boundaries.append(LineObject(self))
		else: 
			for i in range(len(sequence_points)):
				self.boundaries.append(LineObject(self,sequence_points[i]))

	def getPoints(self):
		""" returns the sequence of line segments that define the object """ 
		return [l.getPoints() for l in self.boundaries]

	def getColor(self):
		""" returns the color of the object """ 
		return self.color 

class SensorObject(VisualObject):
	def __init__(self,sequence_points = [], color = "green"):
		VisualObject.__init__(self,sequence_points,color)

	def drawObject(self,guiobj):
		""" draws the visual object on the canvas referenced by guiboj """
		line_segments = self.getPoints()
		object_color  = self.getColor()
		for line in line_segments:
		    p1 = line[0]
		    p2 = line[1]
		    guiobj.displayLine(p1,p2,fill = object_color) 

		radius = 3 
		x0,y0 = p1 
		top_left_corner     = x0 - radius, y0 + radius 
		bottom_right_corner = x0 + radius, y0 - radius  

		guiobj.displayOval(top_left_corner,bottom_right_corner,fill = "red")	


class ParticleObject(VisualObject):
	def __init__(self,position = (0,0),color = "blue"):
		self.position = (0,0) 
		self.d =  0.8
		self.setPosition(position)
		seq = self.constructParticleBoundaries()
		VisualObject.__init__(self,seq,color)

	def constructParticleBoundaries(self):
		x0,y0 = self.getPosition()
		d  = self.d 
		sequence = [[(x0 - d, y0 + d), (x0 + d, y0 + d)],
					[(x0 + d, y0 + d), (x0 + d, y0 - d)],
					[(x0 + d, y0 - d), (x0 - d, y0 - d)],
					[(x0 - d, y0 - d), (x0 - d, y0 + d)]]
		return sequence 

	def drawObject(self,guiobj):
		""" draws the visual object on the canvas referenced by canvas """ 
		# line_segments = self.getPoints()
		object_color  = self.getColor()
		x0, y0 = self.getPosition()
		d = self.d 
		p1 = (x0 - d, y0 + d)
		p2 = (x0 + d, y0 - d)
		# guiobj.displayRectangle(p1,p2,fill = object_color) 
		guiobj.displayOval(p1,p2,fill = object_color)

	def setPosition(self,position):
		self.position = position 
	def getPosition(self): # returns the particle position 
		return self.position 



def testPointObject():
	seq_list = [(0,0), (1,0), (1,1), (0,1)]

	p1 = PointObject(None, seq_list[0])
	assert(p1.getPoints() == seq_list[0])

	p1.setPoints(seq_list[2])
	assert (p1.getPoints() == seq_list[2])

	print("testPoints success")

def testLineObject():
	seq_list = [(0,0), (1,0)]
	seq_list3 = [ (1,1), (0,1)]
	seq_list2 = [(0,0), (1,0)]
	seq_list4 = [(1,1.5), (0,1.2)]

	l1 = LineObject(None, seq_list)
	points = l1.getPoints() 
	assert(points == seq_list)

	l2 = LineObject(None, seq_list) 
	l2.setPoints(seq_list2)
	points2 = l2.getPoints()
	assert(points2 == seq_list2)

	print("testLineObject success")

def testVisualObject():
	seq_list = [[(0,0), (1,0)],[(1,0),(1,1)], [(1,1), (0,1)]]

	output_seq = [[(0,0),(1,0)],[(1,0),(1,1)],[(1,1),(0,1)]]

	vo1 = VisualObject(seq_list)
	points = vo1.getPoints()
	assert (output_seq == points)
	print ("testVisualObject success")





def main():
	testPointObject()
	testLineObject()
	testVisualObject()



if __name__ == '__main__':
	main() 


