#!/usr/bin/env python

"""
Vishal Kole
Mounika Alluri
Shih Ting 

Particle filter for localization with 6 initial probable locations.
"""
import rospy, cv2 as cv, matplotlib.image as mpimg, numpy as np, math
from mapGUI import Mapper
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
import numpy as np
from random import randint
import Tkinter as tk
from PIL import Image
import ImageTk
import random
import rospy
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

WIDTH = 2000
HEIGHT = 700
PARTICLE_SIZE = 10

class Pose():

	def __init__(self,x,y,t):
		self.x = x
		self.y = y
		self.theta = t


class Particle():

	def __init__(self,x,y,t,w):
		self.pose = Pose(x,y,t)
		self.weight= w


class ParticleFilter():


	def __init__(self,m):

		self.mapper = m
		self.odom = None
		self.laser= None
		self.sonar = None
		self.previous_odom = None
		self.total_particles = None
		self.particles = list()
		self.twist = Twist()
		self.init_with_values()
		self.safe_wander()
		self.pub_vel = rospy.Publisher('/r1/cmd_vel', Twist, queue_size=10)
		self.pub_pose = rospy.Publisher('/r1/localization', String, queue_size = 10)
	

	def init_with_values(self):

		area = np.arange(-0.8, 0.8, 0.1)
		locations =  [(8.0, -0.5, math.pi/2), (-12.0, 	12.0, math.pi), (-18.4, -8.9, 0), \
			   (10.8, 12.7, math.pi),(-54.5, 7.6, math.pi/2), (8.0, -1.5,-math.pi/2)]
		theta = [ -0.2, -0.1 , 0, 0.1, 0.2 ]

		for i in locations:
			for x in area:
				for y in area:
					for t in theta:
						self.particles.append(Particle(i[0]+x,i[1]+y,i[2]+t,0.5))

		self.total_particles = len(self.particles)


	def safe_wander(self):
		"""
		move forward without running into obstacles until convergence
		"""
	
		safe_dist = 0.8
		if self.sonar.ranges[0] > safe_dist  and self.sonar.ranges[7] >safe_dist:	
			self.twist.linear.x = 0.1
		elif self.sonar.ranges[0] <safe_dist:
			self.twist.angular.z = 0.1
		elif self.sonar.ranges[7] <safe_dist:
			self.twist.angular.z = -0.1
		else:
			print("I'm trapped")
		
			
	def predict(self):
		"""
		compute delta(odom) and apply to each particle
		"""

		if self.previous_odom is not None:
			# current robot position
			xr     = self.odom.pose.pose.position.x - self.previous_odom.pose.pose.position.x
			yr     = self.odom.pose.pose.position.y - self.previous_odom.pose.pose.position.y
			z      = self.odom.pose.pose.orientation.z
			zr     = self.previous_odom.pose.pose.orientation.z
			w      = self.odom.pose.pose.orientation.w
			wr     = self.previous_odom.pose.pose.orientation.w
			theta  = 2*math.atan2(z,w)
			theta2 = 2*math.atan2(zr,wr)
			thetar = theta - theta2
			dist   = math.sqrt(xr**2 + yr**2)

			for particle_number in range(len(self.particles)):

				self.particles[particle_number].pose.theta += thetar
				[xn ,yn ] = self.angleToXY(self.particles[particle_number].pose.theta,dist)
				if self.mapper.isPixelInMap(self.particles[particle_number].pose.x+xn, self.particles[particle_number].pose.y+yn):
					self.particles[particle_number].pose.x += xn
					self.particles[particle_number].pose.y += yn
				else:
					self.particles[particle_number].weight = 0.0

		self.previous_odom = self.odom


	def angleToXY(self, theta, dist):
		"""
		Convert odom to 
		"""
		x = dist * math.cos(theta)
		y = dist * math.sin(theta)
		return [x, y]


	def update(self):
		"""
		Update the particle weights based on sensor data
		"""

		left_sensor = self.sonar.ranges[0]
		right_sensor = self.sonar.ranges[7]
		center_sensor = self.laser.ranges[320]

		if center_sensor == float("inf") or center_sensor >8.0:
			center_sensor = 8.0 # max distance from laser
		threshold = 0.1
		percent_change = 1.1

		for item in self.particles:
			particle_reading = self.mapper.getReading(item.pose.x,item.pose.y,item.pose.theta)
			if math.fabs(left_sensor - particle_reading[0]) <threshold :
				item.weight *= percent_change
			else:
				item.weight /= percent_change
			if math.fabs(right_sensor - particle_reading[2]) <threshold:
				item.weight *= percent_change
			else:
				item.weight /= percent_change
			if math.fabs(center_sensor - particle_reading[1]) <threshold:
				item.weight *= percent_change
			else:
				item.weight /= percent_change
				
		new = list()
		for item in self.particles:
			if item.weight > 0.3:
				new.append(item)
		self.particles = new
		self.mapper.particle_update(self.particles)


	def resample(self):
		"""
		Delete particles with lower weights and duplicate particles with higher weights
		"""
		theta = [0, -0.1 , 0, 0.1, 0]
		self.particles.sort(key=lambda x: -x.weight)

		if self.total_particles - len(self.particles) > 0:
			new = list(self.particles)
			#print("total particles ="+str(self.total_particles))
			#print("iteration ="+str((self.total_particles - len(self.particles))))
			#print("length of particles"+str(len(self.particles)))

			for j in range(self.total_particles - len(self.particles)):
				i = j% (len(self.particles))
				new.append(Particle(self.particles[i].pose.x + random.uniform(0.1,0.2) , self.particles[i].pose.y + random.uniform(0.1,0.2), self.particles[i].pose.theta + random.choice(theta),  self.particles[i].weight ))
				#new.append(Particle(self.particles[i].pose.x, self.particles[i].pose.y, self.particles[i].pose.theta, self.particles[i].weight ))
			#print("old = " +str(len(self.particles))+" new = " + str(len(new)))
			self.particles = new


	def converge(self):
		"""
		 Estimate pose of robot as mean of particles if sum of squared erros is less than 10  
		"""
		threshold_converge = 10
		center = Pose(0, 0, 0)
		sumX, sumY, sumT = 0, 0, 0
		n = len(self.particles)
		for p in self.particles:
			sumX += p.pose.x
			sumY += p.pose.y
			sumT += p.pose.theta
		center.x, center.y = sumX/n, sumY/n

		for i in range(n):
			p = self.particles[i]
			dist = math.sqrt(math.pow((p.pose.x - center.x),2) + math.pow((p.pose.y - center.y), 2))
			if dist > threshold_converge:
				print("center"+ str(i) + ': (' + str(center.x) + ',' + str(center.y) + ')')
				return
		print(len(self.particles))
		self.particles = Particle(center.x, center.y, sumT/n, 1)
		pub_pose.publish(str(self.particles.pose))


	def odom_callback(self,data):
		self.odom= data
		self.predict()
		self.update()
		self.resample()
		self.converge()


	def laser_callback(self,data):
		self.laser = data


	def sonar_callback(self,data):
		self.sonar = data


def main():

	root = tk.Tk()
	m = Mapper(master=root,height=WIDTH,width=HEIGHT)
	pf = ParticleFilter(m)
	
	# initialize nodes and subscribers
	rospy.init_node("particle_filter", anonymous=True)
	rospy.Subscriber("/r1/odom", Odometry, pf.odom_callback, queue_size = 1)
	rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, pf.laser_callback, queue_size = 1)
	rospy.Subscriber("/r1/sonar", SonarArray, pf.sonar_callback, queue_size = 1)
	root.mainloop()

if __name__== "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		print("rospy.ROSInterruptException")
