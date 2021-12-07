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
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

NAME = "Cadena Campos Luis -Flores Gonzalez Jesus- Hernandez FontesAldo"

def mover_robot():
    #Obtener donde esta ubicado el robot 
    coord_pos=PoseStamped()
    global candado
    if not candado:
        if(orden=="JUSTINA MOVE ANNEXED"):
            hablar("MOVING TO THE ANNEXED")
            #ANEXO (0,0)
            coord_pos.pose.position.x=(0) 
            coord_pos.pose.position.y=(0)
            publishing_coord.publish(coord_pos)
            if meta:
                hablar("Arrived to the ANNEXED")
            else meta:
                hablar("JUSTINA is moving to the ANNEXED")
    
        elif(orden=="JUSTINA MOVE PRINCIPAL"):
            hablar("MOVING TO THE PRINCIPAL")
            #PRINCIPAL (5.28104,4.05978)
            coord_pos.pose.position.x=(5.28104) 
            coord_pos.pose.position.y=(4.05978)
            publishing_coord.publish(coord_pos)
            if meta:
                hablar("Arrived to the PRINCIPAL")
            else meta:
                hablar("JUSTINA is moving to the PRINCIPAL")

        elif(orden=="JUSTINA MOVE INSTITUTE"):
            hablar("MOVING TO THE INSTITUTE")
            #INSTITUTE (8,0)
            coord_pos.pose.position.x=(9) 
            coord_pos.pose.position.y=(5)
            publishing_coord.publish(coord_pos)
            if meta:
                hablar("Arrived to the INSTITUTE")
            else meta:
                hablar("JUSTINA is moving to the INSTITUTE")        
    else 
        print("En movimiento")


def hablar(texto):
    print texto
    Robot_voice=SoundRequest()
    Robot_voice.sound= -3
    Robot_voice.volume= 1
    Robot_voice.command= 1
    Robot_voice.arg= texto

    publicador_habla.publish(Robot_voice)


def cachar_orden():
    #Ob
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
    global loop,publishing_coord,publicador_habla
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
    
