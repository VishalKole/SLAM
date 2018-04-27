#!/usr/bin/env python

"""
Vishal Kole
Mounika Alluri

Go to specified co-ordinates with obstacle avoidance
(Tested on Pioneer P3-DX, ROS Kinetic with Ubuntu 16.04)

"""
import rospy
import time
import math
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from p2os_msgs.msg import MotorState

#global data
pub = None
odom = None
scan = None
coordinates = None
twist_obj = None
hasObstacle = None
obs_loc = None
angle_difference = None
right_done = None
right_done = None
wall_follow = None
sonar = None
sensor_active = None
pub_motor= None

#get the coordinates
def instantiate_fileCoordinates():
	global coordinates
	coordinates = list()
	file = open(str(sys.argv[1]),"r")
	for line in file:
		line_split = line.strip().split()
		coordinates.append([float(line_split[0]),float(line_split[1])])

def instantiate_parameters():
	global twist_obj
	global angular_speed
	global linear_speed
	global hasObstacle
	global angle_difference
	global right_done
	global wall_follow
	global left_done
	global sensor_active
	sensor_active = " "
	wall_follow = False
	right_done = False
	left_done = False
	hasObstacle = False
	angle_difference = 0
	twist_obj = Twist()

def instantiate_publisher():
	global pub
	global pub_motor		
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub_motor= rospy.Publisher('/cmd_motor_state',MotorState, queue_size=10)

def getSonar(data):
	global sonar
	sonar = data

def instantiate_subscribers():
	#sub_scan = rospy.Subscriber('/kinect_laser/scan', LaserScan, callback_scan)
	rospy.Subscriber('/sonar',SonarArray, getSonar)
	rospy.Subscriber("/pose", Odometry, callbackOdom)	


def publish_twist(angular=0,linear=0):
	global twist_obj
	global pub
	twist_obj.angular.z = angular
	twist_obj.linear.x = linear
	pub.publish(twist_obj)
	
#the basic walk function
def walk():
	global coordinates
	global odom
	global hasObstacle
	global angle_difference
	global sonar
	global scan
	global obs_loc
	global right_done
	global wall_follow
	global left_done
	global sensor_active

	angle_threshold = 0.1
	zero = 0.0
	error_dist = 0.03
	
	if not coordinates:
		rospy.loginfo("No Co-ordinates to go. Shutting down!")
		rospy.signal_shutdown("No Co-ordinates to go. Shutting down!")
	
	cur_x = odom.pose.pose.position.x
	cur_y = odom.pose.pose.position.y
	cur_z = odom.pose.pose.orientation.z
	cur_w = odom.pose.pose.orientation.w
        
	cur_theta = 2*math.atan2(cur_z,cur_w)
	if cur_theta> math.pi:
		cur_theta = ((math.pi) + ( math.pi - cur_theta))*-1
	if cur_theta< -1*math.pi:
		cur_theta = ((math.pi) - (( math.pi + cur_theta)*-1))
	
	goal_theta = math.atan2((coordinates[0][1]-cur_y),(coordinates[0][0]-cur_x))
	angle_diff = goal_theta - cur_theta
	dist = math.sqrt((coordinates[0][0] - cur_x)**2 + (coordinates[0][1] - cur_y)**2)  
	angle_difference = angle_diff

	
	print("____________________________________\n" +
	"x "+str(cur_x)+"\n"+
	"y "+str(cur_y)+"\n"+
	"current "+str(cur_theta)+"\n"+
	"goal "+str(goal_theta)+"\n"+
	"angle_diff "+str((angle_diff))+"\n"+
	"dist "+str(dist)+"\n"+
	"Sonar left: " + str(sonar.ranges[0])+"\n"+
	"Sonar right: " + str(sonar.ranges[7])+"\n"+
	"Sonar left center: " + str(sonar.ranges[3])+"\n"+
	"Sonar right center: " + str(sonar.ranges[4])+"\n"+
	"obsloc: " + str(obs_loc)+"\n"+
	"right_done: " + str(right_done)+"\n"+
	"left_done: " + str(left_done)+"\n"+
	"wall_follow: " + str(wall_follow)+"\n"+
	"sensor_active: " + str(sensor_active))
	

	#turning function
	if(math.fabs(angle_diff)>angle_threshold and math.fabs(dist)>error_dist) and not hasObstacle:
		if (angle_diff>0 and math.fabs(angle_diff)<math.pi):
			publish_twist( 0.1,0)
		elif(angle_diff>0 and math.fabs(angle_diff)>=math.pi):
			publish_twist( -0.1,0)		
		elif(angle_diff<0 and math.fabs(angle_diff)<math.pi):
			publish_twist(-0.1,0)
		else:
			publish_twist(0.1,0)			

	elif(math.fabs(dist)>error_dist) and not hasObstacle:
		publish_twist(0,0.25)

	elif not hasObstacle:
		publish_twist(0,0)
		rospy.loginfo("********Reached!**********")
		coordinates.pop(0)

		if not coordinates:
			rospy.loginfo("********* Done, Shutting down **********")
			rospy.signal_shutdown('Shut Down')
                  


def callbackOdom(data):
	global pub
	global odom
	global pub_motor
	odom = data
	rate = rospy.Rate(10)
	rate.sleep()
	pub_motor.publish(1)
	walk()
	obstacle_detection()
	maneuver_robot()


def callback_scan(data):
	global scan 
	scan = data
	
#avoids the obstacles and goes around it
def maneuver_robot():
	global hasObstacle	
	global obs_loc
	global odom
	global angle_difference
	global right_done
	global wall_follow
	global sonar
	global scan
	global left_done
	global sensor_active
	
	if hasObstacle and (sonar.ranges[0] > 0.5) and right_done == False and wall_follow == False and obs_loc == "left":
		publish_twist(-0.1,0.0)
	elif hasObstacle and (sonar.ranges[7] > 0.5) and left_done == False and wall_follow == False and obs_loc == "right":
		publish_twist(0.1,0.0)
	elif hasObstacle and right_done == False and wall_follow == False:
		publish_twist(0.0,0.0)
		right_done = True
		wall_follow = True
		sensor_active = "left"
	elif hasObstacle and left_done == False and wall_follow == False:
		publish_twist(0.0,0.0)
		left_done = True
		wall_follow = True
		sensor_active = "right"
		

		#if the obstacle is to the left
	if wall_follow == True and sensor_active == "left":	
		if sonar.ranges[0] <0.5 :
			publish_twist(-0.08,0.1)
		elif sonar.ranges[0] >0.6 :
			publish_twist(0.15,0.15)
		elif sonar.ranges[0] >0.52 :
			publish_twist(0.08,0.1)
		else :
			publish_twist(0.0,0.15)

		#if the obstacle is to the left
	if wall_follow == True and sensor_active == "right":	
		if sonar.ranges[7] <0.5 :
			publish_twist(0.08,0.1)
		elif sonar.ranges[7] >0.6 :
			publish_twist(-0.15,0.15)
		elif sonar.ranges[7] >0.52 :
			publish_twist(-0.08,0.1)
		else :
			publish_twist(0.0,0.15)
  
	right_thresh = 0.8

        #wall follow to the left
	if sensor_active == "right" and angle_difference <(math.pi/2)- right_thresh and angle_difference >(math.pi/2)+ right_thresh  :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False

        #wall follow to the right
	if sensor_active == "left" and angle_difference <-(math.pi/2)- right_thresh  and angle_difference >-(math.pi/2)+ right_thresh :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False
					
		
def obstacle_detection():
	global scan
	global obs_loc
	global hasObstacle
	global sonar
	global right_done
	global wall_follow
	global left_done

	stopping_dist = 0.5
	
	
	#stop if free in the world without obstacle
	if ((sonar.ranges[3] > stopping_dist) and (sonar.ranges[2] > stopping_dist) and (sonar.ranges[1] > stopping_dist) and ((sonar.ranges[4] > stopping_dist) and (sonar.ranges[5] > stopping_dist) and (sonar.ranges[4] > stopping_dist))) and sonar.ranges[7] >0.8 and sonar.ranges[0] >0.8 :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False
	
	
	"""
	if ((sonar.ranges[3] > stopping_dist) and (sonar.ranges[2] > stopping_dist) and (sonar.ranges[1] > stopping_dist) and ((sonar.ranges[4] > stopping_dist) and (sonar.ranges[5] > stopping_dist) and (sonar.ranges[4] > stopping_dist))) and sonar.ranges[7] >1 and sonar.ranges[0] >1 :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False
	"""


	#Continuously check for obstacles
	if   (sonar.ranges[3] < stopping_dist) or (sonar.ranges[2] < stopping_dist) or (sonar.ranges[1] < stopping_dist):
		hasObstacle = True
		left_done = False
		right_done = False
		wall_follow = False
		obs_loc = "left"

	elif (sonar.ranges[6] < stopping_dist) or (sonar.ranges[5] < stopping_dist) or (sonar.ranges[4] < stopping_dist):
		hasObstacle = True
		right_done = False
		left_done = False
		wall_follow = False
		obs_loc = "right"
	else:
		obs_loc = None
	
	
def main():
	instantiate_fileCoordinates()
	instantiate_publisher()
	instantiate_parameters()
	rospy.init_node('Explorer', anonymous=True)
	instantiate_subscribers()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
