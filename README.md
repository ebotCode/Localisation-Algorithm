# Localisation-Algorithm

This Repo contains my project on autonomous vehicle Localistion setting using Particle filter. 

The task was for the vehicle to locate where it is in a given map. The map chosen for this project, was a map of my living room space. 

Localisation is the process by which self-driving cars (or robots) determine where in the world they are in the map of the world that they posses. 

Description: 

VisualDisplay.py generates a visual display of the localisation process. The vehicle is represented as a red circle, while the particles are represented as blue circles. 

Sensors are mocked as LIDAR. we use 8-scan (which is the same as using 8 scans data from a 360 deg LIDAR device.)

The distribution starts out with having particles uniformly distributed over my living rooom space, which represents the vehicles uncertainty as to where in the living room map it is. As the vehicle moves, the particle filter algorithm takes in the sensor measurement and motion commands to update the distribution of the particles, narrowing down to where the vehicle (red circle) is. 

Main: 
	The Driver program for the project is RunLocaliseHome.py

