import sys
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sound_play.msg import SoundRequest
from std_msgs.msg import String

NAME = "Proyecto_Final_Gutierrez_Alcibar_y_Herrera_Cordero"
 
robopub = None

def movement(instruction):
	global moving
	coords = PoseStamped()

	if moving:
		robotalk(instruction)
		print(moving)
	else:
		coords.position.w = 1
		if (instruction == "GO TO BOOKSELLER"):
			coords.pose.position.x = 7.0000
			coords.pose.position.y = -1.0000
			robopub.publsh(coords)
			robotalk("Moving to " + instruction )
		elif(instrucion == "GO TO SAFETY CONE"):
			coords.pose.position.x = 8.5000
			coords.pose.position.y = 0.0000
			robopub.publish(coords)
		elif(instruction == "GO TO TABLE"):
			coords.pose.position.x = 8.8000
			coords.pose.position.y = 8.4500
			robopub.publish(coords)
		elif(instruction == "GO TO COUCH"):
			coords.pose.position.x = 9.0000
			coords.pose.position.y = 6.4000
			robopub.publish(coords)
		else(instruction == "GO TO DESK"):
			coords.pose.position.x = 3.2000
			coords.pose.position.y = 6.2000
			robopub.publish(coords)

def robotalk(talk):
	Rtalk = SoundRequest()
	Rtalk.sound = -3
	Rtalk.volume = 1
	Rtalk.command = 1
	Rtalk.arg = talk

	robopubtalk.publish(Rtalk)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptionexception:
		pass
