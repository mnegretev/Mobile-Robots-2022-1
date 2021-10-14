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

# Biblioteca
import rospy
# Componente para el escaner del robot
from sensor_msgs.msg   import LaserScan
# Componente para hacer movimientos
from geometry_msgs.msg import Twist

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def callback_scan(msg):
    global obstacle_detected
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    # Set the 'obstacle_detected' variable with True or False, accordingly.
    #
    obstacle_detected = msg.ranges[len(msg.ranges)//2] < 1.0
    print obstacle_detected
    return

def main():
    print "PRACTICE 00 - " + NAME
    rospy.init_node("practice00")
    # Se crea un topico al que se desea suscribir, cuando el laser lanza una publicacion, 
    # la se hace el llamado a la funcion 'callback_scan'
    rospy.Subscriber("/scan", LaserScan, callback_scan)
    # Se crea un publicador de los movimientos que se haran.
    # Cuando hay red o recursos disponibles, se envia el mensaje
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # Se debe de poner una variable en Hertz para esperar a que se lancen las publicaciones
    loop = rospy.Rate(10)
    
    global obstacle_detected
    obstacle_detected = False
    # Mientras no haya una instruccion de apagado, haz todo
    while not rospy.is_shutdown():
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front of the robot, and stop otherwise.
        # Use the 'obstacle_detected' variable to check if there is an obstacle. 
        # Publish the Twist message using the already declared publisher 'pub_cmd_vel'.
        #
	msg = Twist()
	msg.linear.x = 0.3 if not obstacle_detected else 0.0
	pub_cmd_vel.publish(msg)
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
