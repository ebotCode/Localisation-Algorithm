# Localisation-Algorithm

This Repo contains my project on autonomous vehicle Localistion setting using Particle filter. 

The task was for the vehicle to locate where it is in a given map. The map chosen for this project, was a map of my living room space. 

Localisation is the process by which self-driving cars (or robots) determine where in the world they are in the map of the world that they posses. 

programming language: Python 
libraries : matplotlib, tkinter, numpy

Description: 

VisualDisplay.py generates a visual display of the localisation process. The vehicle is represented as a red circle, while the particles are represented as blue circles. 

Sensors are mocked as LIDAR. we use 8-scan (which is the same as using 8 scans data from a 360 deg LIDAR device.)

The distribution starts out with having particles uniformly distributed over my living rooom space, which represents the vehicles uncertainty as to where in the living room map it is. As the vehicle moves, the particle filter algorithm takes in the sensor measurement and motion commands to update the distribution of the particles, narrowing down to where the vehicle (red circle) is. 

How do you tell when the algorithm has converged? well, this is my approach. Imagine you are the robot, and you don't have a reference value to compare the result of your algorithm to. That is, you don't know the ground truth. The best estimate of your pose (position and orientation) would be the estimate that generates your measurement, according to the chossen measurement model and process noise model. In other words, when you get a distribution of particles that minimizes how far apart the measurement from your sensors are from the sensor measurement generated, then you can say you know where you are. In this project, the algorithm got as low as 0.88 for the error. Though i set a criteria of about 2cm for position and 1 deg for mean orientation error (actually got less than that). 


Main: 
	The Driver program for the project is RunLocaliseHome.py

Here are some images from the simulation run with 1000 particles: 

After 5 iterations: 
![Alt text](/img/pf_img2.png?raw=true "particle distribution after 5 few runs")

After 8 iterations: 
![Alt text](/img/pf_img3.png?raw=true "particle distribution after 8 few runs")

Subsequent iterations: 

![Alt text](/img/pf_img4.png?raw=true "")

![Alt text](/img/pf_img5.png?raw=true "")

![Alt text](/img/pf_img6.png?raw=true "")

![Alt text](/img/pf_img7.png?raw=true "")

![Alt text](/img/pf_img8.png?raw=true "")

![Alt text](/img/pf_img9.png?raw=true "")

![Alt text](/img/pf_img10.png?raw=true "")

![Alt text](/img/pf_img11.png?raw=true "")

![Alt text](/img/pf_img12.png?raw=true "")

![Alt text](/img/pf_img13.png?raw=true "")


