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
import sys
from sound_play.msg import SoundRequest
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point


NAME = "Cadena Campos Luis -Flores Gonzalez Jesus- Hernandez FontesAldo"

candado = False
meta = False

def mover_robot(orden):
    global candado
    #Obtener donde esta ubicado el robot 
    coord_pos=PoseStamped()
    if not candado:
        if(orden=="JUSTINA MOVE ANNEXED"):
            hablar("MOVING TO THE ANNEXED")
            #ANEXO (0,0) NUEVO MAPA ANEXO (3.3,0.78)
            coord_pos.pose.position.x=(3.3) 
            coord_pos.pose.position.y=(0.78)
            publishing_coord.publish(coord_pos)
            coord_pos=PoseStamped()
            print(coord_pos.pose.position.x)
            print(coord_pos.pose.position.y)
            if (coord_pos.pose.position.x==3.3 and coord_pos.pose.position.y==0.78): 
                hablar("JUSTINA IS MOVING TO THE ANNEXED")
    
        elif(orden=="JUSTINA MOVE PRINCIPAL"):
            hablar("MOVING TO THE PRINCIPAL")
            #PRINCIPAL (5.0,4.0) NUEVO MAPA #(8.33,0.64)
            coord_pos.pose.position.x=(8.33) 
            coord_pos.pose.position.y=(0.64)
            publishing_coord.publish(coord_pos)
            coord_pos=PoseStamped()
            print(coord_pos.pose.position.x)
            print(coord_pos.pose.position.y)
            if (coord_pos.pose.position.x==8.33 and coord_pos.pose.position.y==0.64): 
                hablar("JUSTINA IS MOVING TO THE PRINCIPAL")

        elif(orden=="JUSTINA MOVE INSTITUTE"):
            hablar("MOVING TO THE INSTITUTE")
            #INSTITUTE (8,0) NUEVO MAPA (,)
            coord_pos.pose.position.x=(5.5) 
            coord_pos.pose.position.y=(1.89)
            publishing_coord.publish(coord_pos)
            coord_pos=PoseStamped()
            print(coord_pos.pose.position.x)
            print(coord_pos.pose.position.y)
            if (coord_pos.pose.position.x==5.5 and coord_pos.pose.position.y==1.89): 
                hablar("JUSTINA IS MOVING TO THE INSTITUTE")       
    else: 
        print("En movimiento")


def hablar(texto):
    print texto
    Robot_voice=SoundRequest()
    Robot_voice.sound= -3
    Robot_voice.volume= 1
    Robot_voice.command= 1
    Robot_voice.arg= texto

    publicador_habla.publish(Robot_voice)


def cachar_orden(msg):
    global candado
    orden=msg.data
    mover_robot(orden)
    

def verificar_mov(msg):
    global candado, meta
    if (msg.linear.x != 0 ):
        candado = True
        meta=False
    elif (msg.linear.x==0):
        candado=False
        meta=True

def main():
    global loop,publishing_coord,publicador_habla,candado
    print "Proyecto Final- " + NAME
    rospy.init_node("proyectoF")
    rospy.Subscriber('/recognized',String,cachar_orden)
    rospy.Subscriber('/cmd_vel', Twist,verificar_mov)
    publicador_habla=rospy.Publisher('/robotsound',SoundRequest,queue_size=10)
    publishing_coord=rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=10)
    loop = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
