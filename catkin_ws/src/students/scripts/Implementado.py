#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  PoseStamped
from std_msgs.msg import String
goal_pub=None
Posicion=None
     
def asigdestino(Posicion):
    Puntometa= PoseStamped()
    if(Posicion == "ROBOT COME TO KITCHEN"):
        Puntometa.pose.position.x=4.0
        Puntometa.pose.position.y=5.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Posicion == "ROBOT COME TO BEDROOM"):
        Puntometa.pose.position.x=5.0
        Puntometa.pose.position.y=2.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Posicion == "ROBOT COME TO THE BATHROOM"):
        Puntometa.pose.position.x=0.0
        Puntometa.pose.position.y=0.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)
    elif(Posicion == "ROBOT COME TO STOP"):
        Puntometa.pose.position.x=0.0
        Puntometa.pose.position.y=0.0
        Puntometa.pose.position.z=0.0
        Puntometa.pose.orientation.w=1.0
        goal_pub.publish(Puntometa)



def main():
    global goal_pub, Posicion
 
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node("Implementado")
    rate = rospy.Rate(30)
    rospy.spin()
    
    rospy.Subscriber("/cmd_vel",Twist,callback_movimiento)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
