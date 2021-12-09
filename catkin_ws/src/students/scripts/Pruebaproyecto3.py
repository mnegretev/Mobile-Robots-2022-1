#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  PoseStamped
from std_msgs.msg import String
goal_pub=None
Destino=None
     
def asigdestino(Destino):
    Puntometa= PoseStamped()
    if(Destino == "ROBOT GO TO CONSTANTINOPLE"):
        Puntometa.pose.position.x=5.0
        Puntometa.pose.position.y=5.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Destino == "ROBOT GO TO CYDONIA"):
        Puntometa.pose.position.x=7.0
        Puntometa.pose.position.y=2.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Destino == "ROBOT GO TO EXIT"):
        Puntometa.pose.position.x=0.0
        Puntometa.pose.position.y=0.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)

def callback_voz(msg):
    Destino = msg.data
    asigdestino(Destino)


def main():
    global goal_pub, Destino
    
    rospy.Subscriber("/recognized", String, callback_voz)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node("Pruebaproyecto3")
    rate = rospy.Rate(30)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass