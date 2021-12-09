#!/usr/bin/env python
#
#
# Robots Moviles 2022-1
# Proyecto Final 
# Movimiento del Robot por Comandos de Voz
# 

import sys
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

NAME = "Manzo_Soto__Montoya_Martinez"
pub_cmd_loc = None
loop        = None

def control_orders(instruction):
    global movement
    localization = PoseStamped()
    if not movement:
	localization.pose.orientation.w = 1
        if (instruction == "ROBOT GO TO THE ENTRANCE"):
            localization.pose.position.x = 2.0
	    localization.pose.position.y = 0
	    pub_cmd_loc.publish(localization)

        elif(instruction == "ROBOT GO TO THE KITCHEN"):
	    localization.pose.position.x = 3.0
	    localization.pose.position.y = 10.0
	    pub_cmd_loc.publish(localization)

	elif(instruction == "ROBOT GO TO THE LIVINGROOM"):
	    localization.pose.position.x = 8.0 
	    localization.pose.position.y = 8.0
	    pub_cmd_loc.publish(localization)

def callback_control(msg):
    instruction = msg.data
    control_orders(instruction)

def callback_goal(msg):
    global movement
    if(msg.linear.x != 0):
        movement = True
    elif(msg.linear.x == 0):
	movement = False

def main():
    global pub_cmd_loc, loop, movement
    movement = False
    print "Proyecto Final- " + NAME
    rospy.init_node("proyecto_final")
    pub_cmd_loc = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/recognized', String, callback_control)
    rospy.Subscriber('/cmd_vel', Twist, callback_goal)
    loop = rospy.Rate(20)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
  
