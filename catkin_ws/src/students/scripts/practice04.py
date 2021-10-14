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

NAME = "CADENA CAMPOS LUIS"

pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):  
    #pos robot_x , pos robot_y , robot_a es el angulo, goal_x & goal_y son los puntos meta.
    cmd_vel = Twist()
    #Asiganaremos valores al alpha y beta
    alpha= 0.1 # Valores de 0.1 que se recomendaron en la clase para
    beta= 0.1  # alpha y beta
    #Declaramos nuestras constantes de velocidad
    v_max = 0.5
    w_max = 1.0
    #Error de angulo
    error_a=math.atan2(goal_y-robot_y,goal_x-goal_y) - robot_a  #error_a es angulo de error.
    #Mantener el intervalo entre (-pi, pi]
        #
        # TODO:
        # Implement the control law given by:
        #
    #usaremos la funcion math.pi() escribir el valor de pi en python
    if (error_a > math.pi):           #Cumplimos la condicion 
        error_a=error_a-2*math.pi   #para mantenernos de (-pi,pi]
    if (error_a < -math.pi):
        error_a =error_a+2*math.pi

    v = v_max*math.exp(-error_a*error_a/alpha)       #Velocidad lineal 
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)   #Velocidad angular maximan
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    cmd_vel.linear.x = v    #Se guardan los valores obtenidos con un twist
    cmd_vel.angular.y= w    #ejemplo de la sintaxis obtenido de: https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    
    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # ----------------------------------------------------------------------------------|
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
    #  while 
    #     Calculate control signals v and w and publish the corresponding message
    #     loop.sleep()  #This is important to avoid an overconsumption of processing time
    #     Get robot position
    #     Calculate local error
    #     If local error is less than 0.3 (you can change this constant)
    #         Change local goal point to the next point in the path
    #     Calculate global error
    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    #---------------------------------------------------------------------------------------|

    #Empezamos a poner nuestra variable idx=0
    idx= 0
    #Despues debemos de empezar con el primer elemento de nuestra posicion, es un elemento local
    [local_x,local_y] = path [idx]
    #Ponemos nuestra ultima posicion,con un elemento global, recordando como funcionan la posicion de los indices en python  
    [global_x,global_y] = path[-1] 
    #Obtenemos la posicion de nuestro robot, con las linea que venia comentada:
    [robot_x,robot_y,robot_a]= get_robot_pose(listener) 
    #Necesitamos declarar los errores para nuestro robot, sera un error global y uno local
    global_error = math.sqrt((global_x - robot_x)**2 + (global_y-robot_y)**2)
    local_error = math.sqrt((local_x - robot_x)**2 + (local_y-robot_y)**2)
    #Establecemos la condicion para que nuestro proceso continue, aun cuando el usuario presione ctrl+c
    while global_error > 0.1 and not rospy.is_shutdown(): #dejamos la tolerancia con un valor de 0.1
        pub_cmd_vel.publish(calculate_control(robot_x,robot_y,robot_a,local_x,local_y))
        loop.sleep()
    #Tenemos que calcular la posicion del robot otra vez
        [robot_x,robot_y,robot_a] = get_robot_pose(listener)  #Obtenemos su posicion
        local_error = math.sqrt((local_x - robot_x)**2 + (local_y-robot_y)**2) #obtenemos el error local
    #Entramos a la condicion if, local_error < 0.3
        if local_error <0.3:  #EL error local debe de tener un valor menor a 0.3
           idx+=1             #Aumentamos idx 
           if idx >= len(path):   
            idx=len(path)-1
            [local_x,local_y]=path[idx]
        global_error = math.sqrt((global_x - robot_x)**2 + (global_y-robot_y)**2) 
    pub_cmd_vel.publish(Twist())
     #se publican los datos
    return
    
def callback_global_goal(msg):
    print ("Calculating path from robot pose to" + str([msg.pose.position.x, msg.pose.position.y]))
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    path = rospy.ServiceProxy('/path_planning/smooth_path',SmoothPath)(SmoothPathRequest(path=path)).smooth_path
    print ("Following path with " + str(len(path.poses)) + " points...")
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print ("Global goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def main():
    global pub_cmd_vel, loop, listener
    print ("PRACTICE 04 - " + NAME)
    rospy.init_node("practice04")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
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
    
    
