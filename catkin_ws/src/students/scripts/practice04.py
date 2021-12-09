#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 4 - PATH FOLLOWING
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#

import rospy
import tf
import math
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from custom_msgs.srv import SmoothPath, SmoothPathRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from std_msgs.msg import String
from sound_play.msg import SoundRequest


NAME = "Practica4"

pub_coordenadas = None
pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    
    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #

    #
    # Primero declaramos las variables de nuestras leyes de control.
    # Estas determinan la sensibilidad en los movimientos del robot.
    #
    alpha = 0.5
    beta = 0.5
    v_max = 0.3
    w_max = 0.5

    #
    # A continuacion determinamos el error de angulo y determinamos que
    # hacer en caso de detectarlo.
    #
    angle_error = math.atan2( goal_y - robot_y, goal_x - robot_x) - robot_a
    if angle_error > math.pi:
        angle_error -= 2*math.pi
    if angle_error <= -math.pi:
        angle_error += 2*math.pi

    #
    # A continuacion obtenemos el valor de la velocidad linear y angular
    #
    v =  v_max*math.exp(-angle_error*angle_error/alpha)
    w = w_max*(2/(1 + math.exp(-angle_error/beta)) -1)
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w


    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # Use the calculate_control function to move the robot along the path.
    # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
    # The publisher for the twist message is already declared as 'pub_cmd_vel'
    # You can use the following steps to perform the path tracking:
    #
    # Set local goal point as the first point of the path
    # Set global goal point as the last point of the path
    # Get robot position with [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    # Calculate global error as the magnitude of the vector from robot pose to global goal point
    # Calculate local  error as the magnitude of the vector from robot pose to local  goal point
    #
    # WHILE global error > tol and not rospy.is_shutdown() #This keeps the program aware of signals such as Ctrl+C
    #     Calculate control signals v and w and publish the corresponding message
    #     loop.sleep()  #This is important to avoid an overconsumption of processing time
    #     Get robot position
    #     Calculate local error
    #     If local error is less than 0.3 (you can change this constant)
    #         Change local goal point to the next point in the path
    #     Calculate global error
    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    #

    #
    # Primero tenemos que asentar las coordenadas del primer y ultimo punto de nuestro arreglo de puntos.
    # Tambien determinamos la posicion y orientacion actual del robot.
    #
    idx = 0
    [local_x, local_y] = path[idx]
    [global_x, global_y] = path[-1]
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)

    #
    # Ahora escribimos las ecuaciones que nos determinan el error global (posicion actual vs final), asi como el
    # error local (posicion actual vs posicion del siguiente punto).
    #
    global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    local_error = math.sqrt((local_x - robot_x)**2 + (global_y - robot_y)**2)

    #
    # Ahora determinamos que va a hacer el robot con base en los errores globales y locales calculados.
    # Agregamos una tolerancia que se va a comparar con el error global para que el while itere mientras.
    #
    while global_error > 0.1 and not rospy.is_shutdown():
        pub_cmd_vel.publish( calculate_control( robot_x, robot_y, robot_a, local_x, local_y))
        loop.sleep()
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
        local_error = math.sqrt((local_x - robot_x)**2 + (local_y - robot_y)**2)

        #
        # Nuevamente utilizamos una tolerancia para determinar si el error local es lo suficientemente chico
        # como para que el programa pase al siguiente punto.
        #
        if local_error < 0.2:
            idx += 1
            if idx >= len(path):
                idx = len(path)-1
            [local_x, local_y] = path[idx]

        global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    pub_cmd_vel.publish(Twist())

    return
    
def callback_global_goal(msg):
    print "Calculating path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y])
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    path = rospy.ServiceProxy('/path_planning/smooth_path',SmoothPath)(SmoothPathRequest(path=path)).smooth_path
    print "Following path with " + str(len(path.poses)) + " points..."
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print "Global goal point reached"

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]


def callback_desplazamiento (msg):
    coordenadas = PoseStamped()
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    if msg == "GO TO A":
        coordenadas.pose.position.x = (3.3)
        coordenadas.pose.position.y = (0.78)
        pub_coordenadas.publish(coordenadas)
        if (robot_x == 5.0 and robot_y == 5.0):
                respuesta("Moving to A")
                
    elif msg == "GO TO B":
        coordenadas.pose.position.x = (8.33)
        coordenadas.pose.position.y = (0.64)
        pub_coordenadas.publish(coordenadas)
        if (robot_x == 3.0 and robot_y == 0.0):
                respuesta("Moving to B")
        
    elif msg == "GO TO C":
        coordenadas.pose.position.x = (5.5)
        coordenadas.pose.position.y = (1.89)
        pub_coordenadas.publish(coordenadas)
        if (robot_x == 7.0 and robot_y == 1.0):
                respuesta("Moving to C")
        
def callback_recognized (msg):
	dato = msg.data
	callback_desplazamiento(dato)
    
def respuesta(msg):
    voice = SoundRequest()
    voice.sound = -3
    voice.volume = 1
    voice.command = 1
    voice.arg = msg
    voice.arg2 = 'voice_kal_diphone'

    pub_respuesta.publish(voice)
    
def main():
    global pub_cmd_vel, loop, listener, pub_coordenadas, pub_respuesta
    print "Proyecto Final - "
    rospy.init_node("practice04")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    rospy.Subscriber('/recognized', String, callback_recognized)
    pub_coordenadas = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_respuesta = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for service for path planning...")
    rospy.wait_for_service('/path_planning/a_star_search')
    print("Service for path planning is now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
