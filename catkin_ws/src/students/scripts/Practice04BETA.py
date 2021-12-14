#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  PoseStamped, Twist
from std_msgs.msg import String
from sound_play.msg import SoundRequest

goal_pub=None
Destino=None
hablar      = None
aux_robot   = 0

def asigdestino(Destino):
    Puntometa= PoseStamped()
    if(Destino == "ROBOT COME TO KITCHEN"):
        Puntometa.pose.position.x=5.0
        Puntometa.pose.position.y=5.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Destino == "ROBOT COME TO BEDROOM"):
        Puntometa.pose.position.x=7.0
        Puntometa.pose.position.y=2.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Destino == "ROBOT COME TO EXIT"):
        Puntometa.pose.position.x=0.0
        Puntometa.pose.position.y=0.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)

def callback_voz(msg):
    Destino = msg.data
    asigdestino(Destino)

def callback_movimiento(Twist):
    movimiento=Twist.linear.x
    robot_hablando(movimiento)



def robot_hablando(movimiento):
    if movimiento>0.1:
        aux_robot=1
        if movimiento<0.1 and aux_robot==1:
            robot_voice=SoundRequest()
            robot_voice.sound=-3
            robot_voice.command=1
            robot_voice.volume=1.0
            robot_voice.arg='goal'
            robot_voice.arg2='voice_kal_diphone'
            hablar.publish(robot_voice)
            aux_robot=0    


def main():
    global goal_pub, Destino
    
    rospy.Subscriber("/recognized", String, callback_voz)
    rospy.Subscriber("/cmd_vel",Twist,callback_movimiento)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    hablar=rospy.Publisher('/robotsound',SoundRequest,queue_size=10)
    rospy.init_node("Practice04BETA")
    rate = rospy.Rate(30)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass