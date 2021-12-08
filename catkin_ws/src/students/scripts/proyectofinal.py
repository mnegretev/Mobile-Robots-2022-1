import sys
import rospy
#import sound_play.msg import SoundRequest
#from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

robopub = None

def movement(instruction):
	global moving
	coords = PoseStamped()

	if moving:
		print(moving)
	else:
		coords.position.w = 1
		if (instruction == "GO TO BOOKSELLER"):
			coords.pose.position.x = 7.0000
			coords.pose.position.y = -1.0000
			robopub.publsh(coords)
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

#def robotalk(talk):
#	Rtalk = SoundRequest()
#	Rtalk.sound = 1
#	Rtalk.volume = 2
#	Rtalk.arg = talk

#	robopubtalk.publish(Rtalk)

def lugar(msg):
	instruction =msg.data
	movement(instruction)

def main():

	glogal robopub
	rospy,init_node("final")
	robotpub=rospy.Publisher('/move_base_simple/goal',PosesStamped,queue_size=1)
	loop = rospy.Rate(20)
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptionexception:
		pass
