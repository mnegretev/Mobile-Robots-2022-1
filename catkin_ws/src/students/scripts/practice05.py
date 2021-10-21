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

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

listener    = None
pub_cmd_vel = None
pub_markers = None

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
    # v and w are the linear and angular speeds.
    # v_max, w_max, alpha and beta, are design constants.
    # Store the resulting v and w in the Twist message 'cmd_vel'
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #    
    cmd_vel = Twist()
    alpha=0.6
    beta=0.3
    v_max=0.8
    w_max=1
    error_a = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a
    while error_a > math.pi:
        error_a = error_a - 2*math.pi
    while error_a <= -math.pi:
        error_a = error_a + 2*math.pi

    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    
    #Publicando las velocidades
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    alpha = 1
    magnitud = (math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2))
    force_x = (alpha*(robot_x - goal_x)) / magnitud
    force_y = (alpha*(robot_y - goal_y)) / magnitud
    return [0, 0]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    beta = 1
    d0=5
    c=0
    sumX = 0
    sumY = 0

    for i in range(len(laser_readings)):
        
        if laser_readings[i][0] < d0:
            
            if math.isinf(laser_readings[i][0]):
                print("inf")
            else:
                c = c + 1
                primero = (1/laser_readings[i][0])
                segundo = (1/d0)
                resta = primero - segundo
                magnitud = beta * math.sqrt(resta)
                comX = math.cos(laser_readings[i][1]+robot_a) 
                comY = math.sin(laser_readings[i][1]+robot_a)

                sumX = sumX + magnitud*(comX/laser_readings[i][0])
                sumY = sumY + magnitud*(comY/laser_readings[i][0])
    force_x = sumX/c
    force_y = sumY/c

    return [force_x, force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print "Moving to goal point " + str([goal_x, goal_y]) + " by potential fields"    
    loop = rospy.Rate(20)
    global laser_readings

    epsilon = 0.5
    tolerance = 0.1 

    robot_x, robot_y, robot_a = get_robot_pose(listener)
    dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    
    while dist_to_goal > tolerance and not rospy.is_shutdown():

        [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
        [frx, fry] = rejection_force(robot_x, robot_y, robot_a, laser_readings)

        [fx, fy] = [fax + frx, fay + fry]
        [px, py] = [robot_x - epsilon*fx, robot_y -epsilon*fy]

        msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, fx, fy, pub_markers)
        loop.sleep()

        robot_x, robot_y, robot_a = get_robot_pose(listener)
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

    pub_cmd_vel.publish(Twist())
    print("Goal point reached")

    #Ejecutar este launch roslaunch bring_up obs_avoidance.launch

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
    
