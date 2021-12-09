#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from std_msgs.msg import String

pub_local_robot = None
loop = None
zone=0
def move_robot(speech_coord):
    global is_moving, zone
    localization = PoseStamped()

    if not is_moving:
        
        
        if(speech_coord == "ROBOT MOVE TO DOOR"):
            print("ROBOT'S MOVING TO DOOR")
            localization.pose.position.x = 0
            localization.pose.position.y = 0
            localization.pose.orientation.w = 1
            zone=1
            pub_local_robot.publish(localization)

        elif(speech_coord == "ROBOT MOVE TO LIVINGROOM"):
            print("ROBOT'S MOVING TO LIVINGROOM")
            localization.pose.position.x = 0.0
            localization.pose.position.y = 0.0
            localization.pose.orientation.w = 1
            zone=2
            pub_local_robot.publish(localization)

        elif(speech_coord == "ROBOT MOVE TO BEDROOM"):
            print("ROBOT'S MOVING TO BEDROOM")
            localization.pose.position.x = 0.0
            localization.pose.position.y = 0.5
            localization.pose.orientation.w = 1
            zone=3
            pub_local_robot.publish(localization)

        elif(speech_coord == "ROBOT MOVE TO KITCHEN"):
            print("ROBOT'S MOVING TO KITCHEN")
            localization.pose.position.x = 0.0
            localization.pose.position.y = 0.0
            localization.pose.orientation.w = 1
            zone=4
            pub_local_robot.publish(localization)
    else:
        print("KEEP MOVING")        

def callback_control(msg):
    #obtiene mensaje tipo string
    #obtiene el texto del reconocimiento de voz
    speech_coord = msg.data
    move_robot(speech_coord)

def callback_goal(msg):
    #obtiene mensaje tipo twist
    #si la propiedad linea de x es diferente a 0, entonces esta en movimiento
    global is_moving
    if(msg.linear.x != 0):
        is_moving = True
    elif(msg.linear.x == 0):
        is_moving = False
    callback_sound()

def callback_sound():
    global zone, is_moving
    if (is_moving==False):
      soundhandle=SoundClient(blocking=True)
      if(zone==1):
          soundhandle.say('The robot has reached the door')
      elif(zone==2):
          soundhandle.say('The robot has reached the living room')
      elif(zone==3):
          soundhandle.say('The robot has reached the bedroom')
      elif(zone==4):
          soundhandle.say('The robot has reached the kichen')
      
def main():
    global pub_local_robot, loop, is_moving, zone, reached
    is_moving = False
    reached=False
    print("FINAL PRACTICE: Ceballos Equihua - Murrieta Villegas - Reza Chavarria - Valdespino Mendieta")
    rospy.init_node("finalproject")
    pub_local_robot = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/recognized', String, callback_control)
    rospy.Subscriber('/cmd_vel', Twist, callback_goal)
    #rospy.Subscriber('/robotsound',SoundRequest, callback_sound)
    
    loop = rospy.Rate(20)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
