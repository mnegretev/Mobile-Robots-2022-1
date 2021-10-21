#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 5 - OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants alpha and beta to get a smooth movement. 
#

import rospy
import tf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "Manzo_Soto_Jorge_Luis"

listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    #
    # TODO:
    # 
    # Implement the control law given by:   
    #Para comenzar con el programa declaramos nuestras variables que se van a utilizar posteriormente en el modelo de control 
    Alpha = 0.1
    Beta = 0.1
    #Posteriormente, declaramos las variables que vamos a utilizar al momento de ejecutar la velocidad dn nuestro sistema
    v_max = 0.4
    w_max = 0.4
    #Declaramos el error que vamos a tomar en cuenta en nuestro angulo, muy importante 
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    # where error_a is the angle error and
    error_a = (math.atan2(goal_y - robot_y, goal_x - robot_x)) - robot_a
    #Declaramos las condiciones que vamos a tomar en cuenta al ejecutar el programa, v and w are the linear and angular speeds.
    # v_max, w_max, alpha and beta, are design constants.
    if(error_a > math.pi):
	error_a = error_a-2*math.pi
    if(error_a < -math.pi):
	error_a = error_a+2*math.pi
    #Establecemos nuestras leyes de control, estas son utilizadas posteriormente para almacenar un mensaje del sistema
    # Store the resulting v and w in the Twist message 'cmd_vel'
    v = v_max*math.exp(-error_a*error_a/Alpha)
    w = w_max*(2/(1+math.exp(-error_a/Beta))-1)
    #Guardamos mensaje en el objeto de tipo twist
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Inicializamos nuestra variable de fuera de atraccion y comenzamos con el siguiente punto de la operacion
    Fuerza_atraccion = 1
    #Calculamos la fuerza de atraccion la cual es dada por el robot y por el punto al que queremos llegar, asi lo indica
    # Calculate the attraction force, given the robot and goal positions.
    force_x=(Fuerza_atraccion/math.sqrt((robot_x-goal_x)**2+(robot_y-goal_y)**2))*(robot_x-goal_x)
    force_y=(Fuerza_atraccion/math.sqrt((robot_x-goal_x)**2+(robot_y-goal_y)**2))*(robot_y-goal_y)
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force w.r.t. map.
    return [force_x, force_y]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    #
    # TODO:
    # Comenzamos con la inicializacion de las variables que se van a autilizar en esta parte de la operacion
    d0 = 0.8
    beta = 3.5
    mag = 0
    # Comenzamos con los calculos, para esto inicializamos en 0 las fuerzas que vamos a tomnar en cuenta en esta operacion
    [force_x,force_y] = [0,0]
    # Calculate the total rejection force given by the average
    # of the rejection forces caused by each laser reading.
    # laser_readings is an array where each element is a tuple [distance, angle]
    # Comenzamos con los ciclos que se van a encargar del calculo
    for [distance,angle] in laser_readings:
    # Establecemos condicionales para el calculo 
      if distance < d0:
        mag = beta*math.sqrt(1/distance-1/d0)
      else:
        mag=0
    # both measured w.r.t. robot's frame.
    # See lecture notes for equations to calculate rejection forces.
      force_x += mag * math.cos(angle+robot_a)
      force_y += mag * math.sin(angle+robot_a)
    [force_x,force_y] = [force_x / len(laser_readings), force_y / len(laser_readings)]
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting rejection force w.r.t. map.
    return [force_x,force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print "Moving to goal point " + str([goal_x, goal_y]) + " by potential fields"    
    loop = rospy.Rate(20)
    global laser_readings

    #
    # TODO:
    # Sum of attraction and rejection forces is the gradient of the potential field,
    # then, you can reach the goal point with the following pseudocode
    # Comenzamos mandando un mensaje de la posicion de la localisacion de nuestro robot, tanto en cordenadas de x y de y
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    # Se indica el lugar a donde el robot desea moverse, es decir, el punto en donde el usuario marco el destino 
    print "ROBOT MOVIENDOSE AL PUNTO SELECCIONADO:" + str([goal_x, goal_y])   
    loop = rospy.Rate(10)
    global laser_readings
    # Establecemos nuestras variables y comenzamos con la operacion
    # Set constant epsilon (0.5 is a good start)
    # Set tolerance  (0.1 is a good start)
    ep=0.5
    to=0.1
    # De nuevo, obtenemos la posicion del robot original, pero esta vez con los lasers que se van a utilizar 
    # Move the robot towards goal point using potential fields.
    # Remember goal point is a local minimun in the potential field, thus,
    # it can be reached by the gradient descend algorithm.
    robot_x, robot_y, robot_a = get_robot_pose(listener) 
    # Calculamos la distancia a nuestro punto meta, es decir, el punto que elige el usuario 
    # Get robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calculate distance to goal as math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    dist_to_goal=math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2) 
    # WHILE distance_to_goal_point > tolerance and not rospy.is_shutdown():
    #     Calculate attraction force Fa by calling [fax,fay]=attraction_force(robot_x,robot_y,goal_x, goal_y)
    #     Calculate rejection  force Fr by calling [frx,fry]=rejection_force (robot_x,robot_y,robot_a,laser_readings)
    #     Calculate resulting  force F = Fa + Fr
    #     Calculate next local goal point P = [px, py] = Pr - epsilon*F
    while dist_to_goal>to or rospy.is_shutdown():
        [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y) 
        [frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings) 
        [fx,fy]=[fax+frx,fay+fry] 
        [px,py]=[robot_x-ep*fx,robot_y-ep*fy] 
        #Calculate control signals by calling msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        #Send the control signals to mobile base by calling pub_cmd_vel.publish(msg_cmd_vel)
        #Call draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers)  to draw all forces
	msg_cmd_vel=calculate_control(robot_x,robot_y,robot_a,px,py) 
        pub_cmd_vel.publish(msg_cmd_vel) 
        draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, fx, fy, pub_markers) 
        #Wait a little bit of time by calling loop.sleep()
	loop.sleep() 
        #Update robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
        robot_x, robot_y, robot_a = get_robot_pose(listener) 
        #Recalculate distance to goal position
        dist_to_goal=math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2) 
       # Publish a zero speed (to stop robot after reaching goal point)
    pub_cmd_vel.publish(Twist()) 
    print("SE LLEGO AL PUNTO META")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def callback_scan(msg):
    global laser_readings
    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]

def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    hdr = Header(frame_id="map", stamp=rospy.Time.now())
    mrk = Marker(header=hdr, ns="pot_fields", id=id, type=Marker.ARROW, action=Marker.ADD)
    mrk.pose.orientation.w = 1
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point(x=robot_x, y=robot_y))
    mrk.points.append(Point(x=(robot_x - force_x), y=(robot_y - force_y)))
    return mrk

def main():
    global listener, pub_cmd_vel, pub_markers
    print "PRACTICE 05 - " + NAME
    rospy.init_node("practice05")
    rospy.Subscriber("/scan", LaserScan, callback_scan)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_pot_fields_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
