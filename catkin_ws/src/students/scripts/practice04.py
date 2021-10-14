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

NAME = "Manzo_Soto_Jorge_Luis"

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
    #Comenzamos declarando las variables para nuestros valores de alpha y de beta respectivamente para iniciar con
    # el procedimiento
    A = 0.1
    B = 0.1
    #Posteriormente declaramos nuestras variables de la velocidad, las cuales nos ayudaran a establecer la manera 
    # en que nuestro objeto sera controlado
    v_max = 1.1
    w_max = 1.1
    #Posteriormente, establecemos y declaramos el valor que va a tener nuestro angulo, esto se tomara en cuenta 
    # al momento de utilizar nuestras velocidad 
    error_a = (math.atan2(goal_y - robot_y, goal_x - robot_x)) - robot_a
    # establecemos las condiciones para el menejo de dicho error, tenemos dos casos, el caso de que nuestro 
    # error ya mencionado sea mayor a nuestro pi
    if(error_a > math.pi):
	error_a = error_a-2*math.pi
    # por otro lado tenemos el caso de que nuestro valor del error sea mayor al valor ya mencionado, despues de 
    # esto continuamos con la operacion
    if(error_a < -math.pi):
	error_a = error_a+2*math.pi
    # Establecemos las leyes que van a controlar la velocidad angular de nuestro sistema
    v = v_max*math.exp(-error_a*error_a/A)
    w = w_max*(2/(1+math.exp(-error_a/B))-1)
    # Por ultimo, almacenamos los valores que hemos calculado en nuestra programacion y los mandamos a un twist, 
    # de esta manera regresamos lo calculado a nuestro sistema 
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
    # Inicializamos nuestra variable idx para comenzar con nuestra siguiente parte del sistema
    idx=0
    # Incializamos nuestro primero elemento del sistema, para esto, llamamos nuestra locaclizacion local, es decir 
    # el punto en donde se encuentra nuestro robot
    [local_x,local_y] = path[idx] 
    # Declaramos nuestro ultimo punto del sistema, este es un punto global, es decir el punto de nuestro 
    # mapa que se encuentra hasta el ultimo
    [global_x,global_y] = path[-1] 
    # recibimos la localizacion exacta de nuestro robot con el siguiente comando, con esto, podemos comenzar a 
    # desarrollar nuestro sistema
    [robot_x, robot_y, robot_a] = get_robot_pose(listener) 
    #AL igual que en el caso anterior, tenemos que desarrollar y declarar una serie de errores que de no hacerlo 
    # van a afectar nuestro sistema de manra significativa 
    global_error = math.sqrt((global_x-robot_x)**2+(global_y-robot_y)**2)
    local_error = math.sqrt((local_x-robot_x)**2+(local_y-robot_y)**2)
    # Establecemos la condicion de que el proceso seguira en el caso de que el usuario no termine con la ejecucion
    # es decir, que presione control c 
    while global_error > 0.1 and not rospy.is_shutdown():
    # Procedemos a calcular el control con el siguiente comando, con esto pasamos a almacenarlo y continuamos con 
    # la operacion
        pub_cmd_vel.publish(calculate_control(robot_x,robot_y,robot_a,local_x,local_y)) 
        loop.sleep() 
    #de nuevo obtenemos la posicion del robot 
        [robot_x, robot_y, robot_a] = get_robot_pose(listener) 
        local_error=math.sqrt((local_x-robot_x)**2+(local_y-robot_y)**2)
    #establecemos una condicinal referente a el error de la posicion de nuestro robot
        if local_error < 0.3:
            idx += 1
            if idx >= len(path):
                idx = len(path)-1
    # establecemos un nuevo punto 
            [local_x,local_y]=path[idx]
        global_error=math.sqrt((global_x-robot_x)**2+(global_y-robot_y)**2)
    pub_cmd_vel.publish(Twist())
   #publicamos los datos
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

def main():
    global pub_cmd_vel, loop, listener
    print "PRACTICE 04 - " + NAME
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
    
