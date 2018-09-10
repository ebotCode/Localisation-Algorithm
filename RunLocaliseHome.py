from VisualDisplay import VisualObject, ParticleObject, SensorObject 
from VisualDisplayGUI import DisplayScreen


from RobotClass import Robot 
from SensorClass import Sensor 
from WorldMapClass import WorldMap, Obstacle 
from ParticleFilterClass import ParticleFilter 

import numpy as np 
import random 
import pickle 

PI = np.pi 


def test4():
	""" This Runs the simulation using a living room world. 
		The living room is modeled as a rectangular boundary of size  360 x 360, 
		with obstacles. 
		The sensor is mocked as a set of rays emanating from the robot, each inclined at different a different 
		angle relative to the robot' heading direction.  
		heading direction (0, 30, 60, 80,). 

		The measurements are therefore the set of distances measured by the sensors. 


		The wrold is represented as a list of bounding segments [(start point),(stop point)]
		"""

	bbox = [(0,360),(360,0)]
	

	world_boundaries = [[(0,0),(300,0)],[(300,0),(300,360)],[(360,360),(0,360)],[(0,360),(0,0)]]

	obstacle1_boundaries = [[(0,0),(60,0)],[(60,0),(60,150)],[(60,150),(0,150)],[(0,150),(0,0)]]
	obstacle2_boundaries = [[(  0,150),(90 ,150)],   [(90 ,150),(90 ,180)],  [(90 ,180),(0  ,180)],  [(0  ,180),(0  ,150)]]
	obstacle3_boundaries = [[(  0,225),(30 ,225)],   [(30 ,225),(30 ,300)],  [(30 ,300),(0  ,300)],  [(0  ,300),(0  ,225)]]
	obstacle4_boundaries = [[( 60,330),(120,330)],   [(120,330),(120,360)],  [(120,360),(60 ,360)],  [(60 ,360),(60 ,330)]]
	obstacle5_boundaries = [[(270,270),(270,180)],   [(270,180),(300,180)],  [(300,180),(300,270)],  [(300,270),(270,270)]]
	obstacle6_boundaries = [[(300,150),(300,180)],   [(300,180),(210,180)],  [(210,180),(210,150)],  [(210,150),(300,150)]]
	obstacle7_boundaries = [[(270,150),(270,30 )],   [(270,30 ),(300,30 )],  [(300,30 ),(300,150)],  [(300,150),(270,150)]]
	obstacle8_boundaries = [[(240, 30),(240,0  )],   [(240,0  ),(300,0  )],  [(300,0  ),(300,30 )],  [(300,30 ),(240,30 )]]


	obstacle_boundaries = [obstacle1_boundaries, obstacle2_boundaries,
						   obstacle3_boundaries,obstacle4_boundaries,
						   obstacle5_boundaries,obstacle6_boundaries,
						   obstacle7_boundaries,obstacle8_boundaries]

	obstacles = [Obstacle(item) for item in obstacle_boundaries]
	# :: Create world Object 
	worldmap = WorldMap(bbox,world_boundaries,obstacles)

	# :: Create sensors  Object 
	sensor = Sensor(worldmap,0.0)
	particle_sensor = Sensor(worldmap,5)
	# :: Create main robot 
	robot = Robot((300.0/2,360.0/2),np.pi/2,sensor)
	# :: creaet mean robot 
	mean_robot = Robot((0,0),np.pi, sensor) 
	mean_robot.setNoise(0.0,0.0)
	
	# :; specify particle noise 
	number_of_particles    = 1000
	particle_turn_noise    =  80*PI/180 # 
	particle_forward_noise = 20  #0.05 	
	# :: create Particle Filter Algorithm Object 
	particle_filter_algorithm = ParticleFilter(worldmap,robot,mean_robot,particle_sensor,
											particle_turn_noise,particle_forward_noise,number_of_particles)

	# :: specify the name of the simulation 
	simulation_name = 'localiseHome1_1000'

	# :: Specify simulation type: 0 -> Real time simulation , 1-> Run and save simulation without any display, 2-> Run saved simulation 
	simulation_type = 2 

	if simulation_type == 0 : # Run real time simulation  
		RunRealTimeSimulation(simulation_name,bbox,worldmap,robot,particle_filter_algorithm)

	elif simulation_type == 1: # Run And save  simulation without displaying it 
		RunAndSaveSimulation(simulation_name,bbox,worldmap,robot,particle_filter_algorithm,200)
	elif simulation_type == 2: # Run saved simulation 
		RunSavedSimulation(simulation_name)
	else:
		print("<Invalid simulation type> ")



def RunRealTimeSimulation(simulation_name,bbox,worldmap,robot,particle_filter_algorithm):
	""" This runs the simulation and displays the results in a gui real time """ 
	object_dict = {"world":VisualObject(worldmap.getMap(),color = 'black'),
				   "sensor":SensorObject(color = 'green')}

	# :: create simulation dictionary 
	simulation_dict = {"robot":robot,
					   "worldmap":worldmap,
					   "filter_algorithm":particle_filter_algorithm }

	a = DisplayScreen(bbox = bbox,object_dict = object_dict,simulation_dict = simulation_dict)
	a.runSimulation(10)

	a.mainloop() 	

def RunAndSaveSimulation(simulation_name,bbox,worldmap,robot,filter_algorithm,niterations = 50):
	""" This runs the simulation 'simulation_name' and saves the results at each iteration as a pickle file.
		Note: when specifying simulation_name, do not include the extension (.lb). This will be included 
		in the function below. just specify the file_name e.g localiseHome3_200  """
	simulation_dict = {}
	simulation_dict['worldmap'] = worldmap
	simulation_dict['bbox']  = bbox  
	simulation_dict['robot'] = robot 
	result = []

	for i in range(niterations):
		print("Iteration = ",i)
		sensor_ray_segs,error = filter_algorithm.localise(1) 
		# :: retrieve all particles anc create particles display object
		particles_position = [particle.getPosition() for particle in filter_algorithm.getParticles()]

		result.append([error,sensor_ray_segs,particles_position])

		if i % 1 == 0:  # save the result every 5 iterations 
			simulation_dict['result'] = result 
			with open(simulation_name+'.lb','wb') as f:
				pickle.dump(simulation_dict,f)

		if error < 2.5:
			print("This is the best estimate of where the robot is  ","*"*200)
			# break


	print("Done with simulation")

def RunSavedSimulation(simulation_filename):
	""" This runs the simulation saved inf simulation_filename.lb """ 
	# open file and retrive the object 
	with open(simulation_filename + '.lb','rb') as f: 
		simulation_dict = pickle.load(f)

	worldmap = simulation_dict['worldmap']
	bbox     = simulation_dict['bbox']
	robot    = simulation_dict['robot']

	simulation_result = simulation_dict['result']
	# :: create object_dict 
	object_dict = {"world":VisualObject(worldmap.getMap(),color = 'black'),
				   "sensor":SensorObject(color = 'green')}
	# :: create simulation dictionary 
	simulation_dict = {"robot":robot,
					   "worldmap":worldmap }

	a = DisplayScreen(bbox = bbox,object_dict = object_dict,simulation_dict = simulation_dict)
	a.runSavedSimulation(simulation_result)

	a.mainloop() 	





 


def main():
	# test1()
	test4()

if __name__ == '__main__':
	main()
