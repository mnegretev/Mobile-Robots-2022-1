#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 9 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of an object 
# given a colored point cloud using color segmentation.
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point

NAME = "Herrejon Ramirez Osvaldo"

def segment_by_color(img_bgr, points):
    #
    # TODO:
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    img_hsv=cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    # - Determine the pixels whose color is in the color range of the ball.
    #   Check online documentation for cv2.inRange
    img_bin=cv2.inRange(img_hsv, numpy.array([(25, 200, 50)], numpy.srray([35, 255, 255]))
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    indices=cv2.findNonZero(img_bin)
    [img_x, img_y, a, b]=cv2.mean(indices)
    print([img_x, img_y])
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    #
    [x, y, z, counter]=[0,0,0,0]
    for[[c,r]] in indices:
        xt=points[r,c][0]
        yt=points[r,c][1]
        zt=points[r,c][2]
        if math.isnan(xt) or math.isnan(yt) or math.isnan(zt):
            continue
        [x, y, z, counter]=[x+xt, y+yt, z+zt, counter+1]
    x=x/counter if counter>0 else 0
    y=y/counter if counter>0 else 0
    z=z/counter if counter>0 else 0
    return [img_x,img_y,x,y,z]

def callback_point_cloud(msg):
    global pub_point
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    bgr = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    [centroid_x, centroid_y, x, y, z] = segment_by_color(bgr, arr)
    hdr = Header(frame_id='kinect_link', stamp=rospy.Time.now())
    pub_point.publish(PointStamped(header=hdr, point=Point(x=x, y=y, z=z)))
    cv2.circle(bgr, (int(centroid_x), int(centroid_y)), 20, [0, 255, 0], thickness=3)
    cv2.imshow("Color Segmentation", bgr)
    cv2.waitKey(1)

def main():
    global pub_point
    print "PRACTICE 09 - " + NAME
    rospy.init_node("practice09")
    rospy.Subscriber("/kinect/points", PointCloud2, callback_point_cloud)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

