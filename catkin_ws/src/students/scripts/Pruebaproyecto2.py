#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  PoseStamped
from std_msgs.msg import String
 
def callback(data):
     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
     
def listener():
 
      rospy.init_node('listener', anonymous=True)
 
      rospy.Suscriber("/recognized", String, callback) 

      rospy.spin()
def talker():
 	goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
	rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
	if listener() == ´ROBOT GO TO KITCHEN´:
		goal_pub.pose.position.x=5.0
		goal_pub.pose.position.y=5.0
		goal_pub.pose.position.z=0.0
		goal_pub.pose.orientation.w=1.0