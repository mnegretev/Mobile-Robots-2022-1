#!/usr/bin/env python

#from mensajes de voz importar mensajes de voz
#para que cuando haya algun reconocimiento de voz puvlique algo y haga algo
#importar el mensaje para mover al rovot 
#importar el mensaje para decirle al rovot que havle cuando llegue a su destino

import rospy
import tf
import math
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from std_msgs.msg import String
from sound_play.msg import SoundRequest

#rot = [0,0,0]

#cocina = ([9, 5, 0], rot) 
#puerta = ([0, 0, 0], rot)
#sala = ([7, 0, 0], rot)
posR = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
nombrePublicador = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
#rospy.Subscriber("recognized", String, callback)

#listener = None
#publisher_voz = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)


#def get_robot_pose(listener):
 #   try:
#	print "Aqui estoy"
#        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
#        a = 2*math.atan2(rot[2], rot[3])
#        a = a - 2*math.pi if a > math.pi else a
#        return [x, y, a]
#    except:
#        pass
#    return [0,0,0]


#def vozRobot(palabra):
#    audio = SoundRequest()
#    audio.sound = -3
#    audio.volume = 1
#    audio.command = 1
#    audio.arg = palabra
#    audio.arg2 = 'voice_kal_diphone'
#    publisher_voz.publish(audio)
def robot_stop():
    stop = Twist()
    stop.linear.x = 0.00
    stop.linear.y = 0.00
    nombrePublicador.publish(stop)

def callback(msg):

    mensaje = msg.data
    coordenada = PoseStamped()
    loop = rospy.Rate(10)
    stop = Twist()
    while not rospy.is_shutdown():
       
        if mensaje == "ROBOT STOP":
            #ROBOT SE DETIENE 
	    robot_stop()
            #print "ROBOT SE DETUVO"
	    break 
  
        elif mensaje == "ROBOT NAVIGATE TO KITCHEN":
            #ROBOT VA A KITCHEN 
            coordenada.pose.position.x=9.00 
	    coordenada.pose.position.y=5.00
            posR.publish(coordenada)
#	    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
#            if coordenada.pose.position.x==9.00 and coordenada.pose.position.y==5.00:
#	        vozRobot("ROBOT NAVIGATE TO KITCHEN")
#		print "ROBOT Llegue a KITCHEN"
      	        robot_stop()
            print "ROBOT VA A KITCHEN"
	    break

        elif mensaje == "ROBOT NAVIGATE TO DOOR":
            #ROBOT VA A DOOR 
            coordenada.pose.position.x=0.00 
	    coordenada.pose.position.y=0.00 
            posR.publish(coordenada)
#	    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
#            if robot_x == coordenada.pose.position.x and robot_y == coordenada.pose.position.y:
#	        vozRobot("ROBOT NAVIGATE TO DOOR")
            print "ROBOT VA A DOOR"
            break

        elif mensaje == "ROBOT NAVIGATE TO LIVINGROOM":
            #ROBOT VA A LIVINGROOM 
            coordenada.pose.position.x=7.00 
	    coordenada.pose.position.y=0.00
            posR.publish(coordenada)
#	    print "ROBOT VA A LIVINGROOM"
#	    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
#            if robot_x == coordenada.pose.position.x and robot_y == coordenada.pose.position.y:
#	        vozRobot("ROBOT NAVIGATE TO LIVNINGROOM")
            print "ROBOT VA A LIVINGROOM"
            break
        loop.sleep()

def main():
    print "FINAL PROYECT"
    rospy.init_node("proyecto_final")
    listener = tf.TransformListener()
    rospy.Subscriber("recognized", String, callback)
    #posR = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    #nombrePublicador = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    #loop = rospy.Rate(10)

    #while not rospy.is_shutdown():
      	#rospy.Subscriber("recognized", String, callback)
    	#posR = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    	#pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #loop.sleep()
    rospy.spin()
        

if __name__== '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass




