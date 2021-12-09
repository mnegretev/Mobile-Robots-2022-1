#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sound_play.msg import SoundRequest
from std_msgs.msg import String

NAME = "Proyecto_Final_Gutierrez_Alcibar_y_Herrera_Cordero"
moving = False 
robopub = None
loop = None

def move_robot(instruction):
	global moving
	coords = PoseStamped()

	if moving == True:
		robotalk(instruction)

	if not moving:
		if (instruction == "GO TO BOOKSELLER"):
			coords.pose.position.x = 5.5000
			coords.pose.position.y = 1.8900
			robopub.publish(coords)
			robotalk("Moving to " + instruction )
		elif(instruction == "GO TO SAFETY CONE"):
			coords.pose.position.x = 0.0000
			coords.pose.position.y = 0.0000
			robopub.publish(coords)
			robotalk("Moving to " + instruction)
		elif(instruction == "GO TO TABLE"):
			coords.pose.position.x = 3.0000
			coords.pose.position.y = 3.0000
			robopub.publish(coords)
			robotalk("Moving to " + instruction)
		elif(instruction == "GO TO COUCH"):
			coords.pose.position.x = 8.3300
			coords.pose.position.y = 0.6400
			robopub.publish(coords)
			robotalk("Moving to " + instruction)
		#elif(instruction == "GO TO DESK"):
		#	coords.pose.position.x = 3.2000
		#	coords.pose.position.y = 6.2000
		#	robopub.publish(coords)
		#	robotalk("Moving to " + instruction)


def robotalk(talk):
	Rtalk = SoundRequest()
	Rtalk.sound = -3
	Rtalk.volume = 1
	Rtalk.command = 1
	Rtalk.arg = talk

	robopubtalk.publish(Rtalk)

def lugar(msg):
	instruction=msg.data
	move_robot(instruction)

def driver(msg):
	if(msg.linear.x != 0):
		moving = False
	elif (msg.linear.x == 0):
		moving = True

def main():

	global robopub,robopubtalk,loop
	rospy.init_node("final")
	rospy.Subscriber('/recognized',String,lugar)
	rospy.Subscriber('cmd_vel', Twist,driver)
	robopubtalk = rospy.Publisher('/robotsound',SoundRequest,queue_size=10) 
	robopub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)
	loop = rospy.Rate(20)
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
