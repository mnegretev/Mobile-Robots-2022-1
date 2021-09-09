#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 0 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot forward until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def callback_scan(msg):
    global obstaculo
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    # Set the 'obstacle_detected' variable with True or False, accordingly.
    obstaculo = msg.ranges[len(msg.ranges)//2] < 1.0
    return

def main():
    print("PRACTICE 00 - " + NAME)
    rospy.init_node("practice00")
    rospy.Subscriber("/scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    
    global obstaculo
    obstaculo = False
    velocidad = Twist()
    while not rospy.is_shutdown():
        
        velocidad.linear.x = 0 if obstaculo else 0.5
        pub_cmd_vel.publish(velocidad)
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
