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

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

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
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    max_w = 1.0
    max_v = 0.7
    alpha=0.1
    beta=0.1

    error_a = (math.atan2(goal_y - robot_y, goal_x - robot_x)) - robot_a

    while error_a > math.pi:
        error_a -= 2*math.pi
    
    while error_a <= -math.pi:
        error_a += 2*math.pi

    v = max_v * math.exp(-error_a*error_a/alpha)
    w = max_w * math.exp(2/(1+math.exp(-error_a/beta))-1)

    while w > math.pi:
        w -= 2*math.pi

    while w <= -math.pi:
        w += 2*math.pi

    cmd_vel.linear.x = v
    cmd_vel.angular.z = w

    return cmd_vel

def follow_path(path):
    #

    i = 0
    tol = 0.1

    # TODO:
    # Use the calculate_control function to move the robot along the path.
    # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
    # The publisher for the twist message is already declared as 'pub_cmd_vel'
    # You can use the following steps to perform the path tracking:

    # Set local goal point as the first point of the path
    [local_x, local_y] = path[0]
    # Set global goal point as the last point of the path
    [global_x, global_y] = path[-1]
    # Get robot position with [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    # Calculate global error as the magnitude of the vector from robot pose to global goal point
    global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    # Calculate local  error as the magnitude of the vector from robot pose to local  goal point
    local_error = math.sqrt((local_x - robot_x)**2 + (local_y - robot_y)**2)
    if local_error < 0.3:
        i +=1
        if i >= len(path):
            i = len(path)-1
        [local_x, local_y] = path[i]
    #
    # WHILE global error > tol and not rospy.is_shutdown() #This keeps the program aware of signals such as Ctrl+C
    while global_error > tol and not rospy.is_shutdown():
    #     Calculate control signals v and w and publish the corresponding message
        pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, local_x, local_y))
    #     loop.sleep()  #This is important to avoid an overconsumption of processing time
        loop.sleep()
    #     Get robot position
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    #     Calculate local error
        local_error = math.sqrt((local_x - robot_x)**2 + (local_y-robot_y)**2)
        if local_error < 0.3:
            i += 1
            if i >= len(path):
                i = len(path) - 1
            [local_x, local_y] = path[i]
        global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    
    pub_cmd_vel.publish(Twist())
    #     If local error is less than 0.3 (you can change this constant)
    #         Change local goal point to the next point in the path
    #     Calculate global error
    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    #

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
    
