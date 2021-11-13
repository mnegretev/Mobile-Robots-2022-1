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

NAME = "Chavez_Contreras_Marco_Antonio"

def segment_by_color(img_bgr, points):
    #
    # TODO:
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the color range of the ball.
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    #   Check online documentation for cv2.inRange
    img_bn = cv2.inRange(img_hsv, numpy.array([25,200,50]), numpy.array([35,255,255]))
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    idx = cv2.findNonZero(img_bn)
    mean_coords = cv2.mean(idx)
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    #
    x,y,z = 0,0,0
    for[[c,r]] in idx:
	if math.isnan(points[r,c][0])&math.isnan(points[r,c][1])&math.isnan(points[r,c][2]):
		return 0
	else:
		x,y,z = x+points[r,c][0], y+points[r,c][1], z+points[r,c][2]
    x,y,z = x/len(idx), y/len(idx), z/len(idx)
    return [mean_coords[0],mean_coords[1],x,y,z]

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

