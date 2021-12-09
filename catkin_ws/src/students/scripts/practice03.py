#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 3 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
#import scipy.linalg
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from custom_msgs.srv import SmoothPath
from custom_msgs.srv import SmoothPathResponse

NAME = "Alfonso Murrieta Villegas"

msg_smooth_path = Path()

def smooth_path(Q, alpha, beta):
    print("Smoothing path with params: " + str([alpha,beta]))
    #
    # TODO:
    # Write the code to smooth the path Q, using the nabla descend algorithm,
    # and return a new smoothed path P.
    # Path is composed of a set of points [x,y] as follows:
    # [[x0,y0], [x1,y1], ..., [xn,ym]].
    # The smoothed path must have the same shape.
    # Return the smoothed path.
    #
    P = numpy.copy(Q)
    tol     = 0.00001                   
    nabla   = numpy.full(Q.shape, float("inf"))
    epsilon = 0.1

    nabla_last = len(nabla) - 1

    nabla[0] = 0
    nabla[nabla_last] = 0

    steps = 0
    nabla_magGrad = numpy.linalg.norm(nabla)

    while nabla_magGrad > tol and steps < 10000:
        for i in range(1, nabla_last):
            nabla[i] = beta * (P[i] - Q[i]) + alpha * (2 * P[i] - P[i-1] - P[i+1])
        P = P - epsilon * nabla
        steps += 1
        if steps % 1000 == 0: # Show progress
            print("Current step: " + str(steps))

    print("Path smoothed correctly!")
    
    return P
    
    ###################################### ORIGINAL ######################################

    # P = numpy.copy(Q)
    # tol     = 0.00001                   
    # nabla   = numpy.full(Q.shape, float("inf"))
    # epsilon = 0.1

    # #PRACTICE 3
    # # Auxiliar values to go inside of while
    # nabla_mag = 1 
    # steps = 1 

    # while nabla_mag > tol and steps < 10000:
        
    #     #Change nabla value to modify after with the normal
    #     nabla_mag = 0
    #     sizeP = len(P) - 1

    #     #Auxiliar for to fill values of position 0 and 1
    #     for i in range(0,1):
    #         nabla[0][i] = alpha* (P[0][i] - Q[0][i]) - beta * (P[1][i]-P[0][i])
    #         P[0][i] = P[0][i] - epsilon * nabla[0][i]

    #     #Algorithm
    #     for i in range(1,sizeP - 1):
    #         for j in range(0,1):
    #             nabla[i][j] = alpha * (P[i][j] - Q[i][j]) + beta * (2*P[i][j] - P[i-1][j] - P[i+1][j])
    #             P[i][j] = P[i][j] - epsilon * nabla[i][j]
            
    #     for i in range(0,1):
    #         nabla[sizeP][i] = alpha * (P[sizeP][i] - Q[sizeP][i]) + beta * (P[sizeP][i] - P[sizeP-1][i])
    #         P[sizeP][i] = P[sizeP][i] - epsilon * nabla[sizeP][i]

    #     #Modify variables to decide if stay on while

    #     # TO NOT install scipy I decided to use numpy
    #     #nabla_mag = scipy.linalg.norm(nabla)
    #     nabla_mag = numpy.linalg.norm(nabla)
    #     steps += 1 
        
    # print("PROCESS HAS DONE")               
    # return P

def callback_smooth_path(req):
    alpha = rospy.get_param('/path_planning/smoothing_alpha')
    beta  = rospy.get_param('/path_planning/smoothing_beta' )
    P = smooth_path(numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in req.path.poses]), alpha, beta)
    msg_smooth_path.poses = []
    for i in range(len(req.path.poses)):
        msg_smooth_path.poses.append(PoseStamped(pose=Pose(position=Point(x=P[i,0],y=P[i,1]))))
    return SmoothPathResponse(smooth_path=msg_smooth_path)

def main():
    print "PRACTICE 03 - " + NAME
    rospy.init_node("practice03", anonymous=True)
    rospy.Service('/path_planning/smooth_path', SmoothPath, callback_smooth_path)
    pub_path = rospy.Publisher('/path_planning/smooth_path', Path, queue_size=10)
    loop = rospy.Rate(1)
    msg_smooth_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_smooth_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


