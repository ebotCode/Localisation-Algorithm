
import numpy as np 

def dot(a,b):
	return np.sum(a * b)

def cross2D(p1,p2):
	return (p1[0]*p2[1] - p2[0]*p1[1])

def isEqual(value1,value2,tol = 1e-10):
	return np.all(np.abs(value1 - value2) <= tol)


def isRayIntersectingLine(ray_origin,ray_direction,line):
	""" returns a tuple of values where the first item is True if the ray 
		intersects the line, false otherwise, and the second item in the 
		coordinate of intersection if True, and [-1,-1] if false. 
		p  = ray_origin 
		(q,b) => position vector of line points. 
		r  = unit vector in direction of ray. r = [cos(ray_direction),sin(ray_direction)]
		s = b - q 
		t = cross2D((q - p),s)/cross2D(r,s)
		u = cross2D((p - q),r)/cross2D(s,r)
		see geometry - What's the most efficent way to calculate where two line segments intersect_ - Stack Overflow

	"""
	# first, compute the unit vector in the direction of the ray 
	
	p = np.array([[ray_origin[0]],[ray_origin[1]]])
	r = np.array([[np.cos(ray_direction)],[np.sin(ray_direction)]])
	q = np.array([[line[0][0]],[line[0][1]]])
	b = np.array([[line[1][0]],[line[1][1]]])
	s = (b - q)

	if abs(cross2D(r,s)) < 1e-8: # This means (case1: collinear (intersect/disjoint), case2: (parrallel))
		# print('-->case of zero denominator')
		return False, np.array([-1,-1])
	else:
		t = cross2D((q - p),s)/cross2D(r,s)
		u = cross2D((p - q),r)/cross2D(s,r)
		if (t >= 0) and (u >=0) and (u <= 1):
			intersection1 = p + r*t
			intersection2 = q + s*u			

			if isEqual(intersection1,intersection2):
				# assert (np.all((intersection1 - intersection2) <= 1e-5))

				return True, intersection1.flatten()
			else:
				# print('-->case of inconsistent intersections')
				return False, np.array([-1,-1])
		else: 
			# print ('-->case of out of range for t1, t2')
			return False, np.array([-1,-1])





isIntersect = isRayIntersectingLine

def test_isIntersect2():
	ray_origin = (4,4)
	ray_direction = 225*np.pi/180 

	# test 1 line intersects with ray_direction > 180
	print('test_isIntersect2--test1')
	line1 = [(1,0),(1,2)]
	bool_ans,value = isIntersect(ray_origin,ray_direction,line1)
	assert ((bool_ans == True) and isEqual(value,np.array([1.,1.])))

def test_isIntersect():
	ray_origin = (0,0)
	ray_direction = np.pi/4

	# test 1 line intersects 
	print('test_isIntersect--test1')
	line1 = [(1,0),(1,2)]
	bool_ans,value = isIntersect(ray_origin,ray_direction,line1)
	assert ((bool_ans == True) and isEqual(value,np.array([1.,1.])))

	# test 2 line is behind the origin of ray. 
	print('test_isIntersect--test2')
	ray_origin = (1,1)
	line2 = [(0,1),(1,0)]
	bool_ans,value = isIntersect(ray_origin,ray_direction,line2)
	assert((bool_ans == False) and isEqual(value,np.array([-1,-1])))

	# test 3 line intersects 
	print('test_isIntersect--test3')
	ray_origin = (0,0)
	line2 = [(0,1),(1,0)]
	bool_ans,value = isIntersect(ray_origin,ray_direction,line2)
	assert((bool_ans == True) and isEqual(value,np.array([0.5,0.5])))	

	# test 4 line segment not intersect: in this case, extending the line segment (infinite actually intersects)
	print('test_isIntersect--test4')
	ray_origin = (0,0)
	line2 = [(0,1),(0.2,0.8)]
	bool_ans,value = isIntersect(ray_origin,ray_direction,line2)
	assert((bool_ans == False) and isEqual(value ,np.array([-1,-1])))
	# assert((bool_ans == True) and np.all(value == np.array([2,2])))	

	# test 6 
	print('test_isInterxect--test6')
	ray_origin = (3.5,2.2)
	ray_directions = [np.pi*i/180 for i in range(0,360,30)]
	segs = [[(1,1),(4,1)],[(1,3),(1,1)],[(1,3),(2.5,3)],[(2.5,3),(2.5,4)],[(2.5,4),(4,4)],
			[(4,4),(4,1)]]
	intersecting_segs = []
	print ('ray directoins = ',[ray_directions[i]*(180/np.pi) for i in range(len(ray_directions))])
	for i in range(len(segs)): 
		line = segs[i]
		for ray_direction in ray_directions:
			bool_ans,value = isIntersect(ray_origin,ray_direction,line)
			if bool_ans == True : 
				intersecting_segs.append(i)
				print ('segment ',line,"value = ", value, "angle = ",ray_direction*180/np.pi)
			# else:
			# 	print ("No intersection ","line = ",line,"angle = ",ray_direction*180/np.pi)

	# print("test_isIntersect success")

def main():
	test_isIntersect()
	print('*'*50)
	test_isIntersect2()


if __name__ == '__main__':
	main()

