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

NAME = "GARCIA OÑATE"

def segment_by_color(img_bgr, points):

    img_hsv=cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
    img_bin=cv2.inRange(img_hsv, numpy.array([25,200,50]),numpy.array([35,255,255]))
    indices= cv2.findNonZero(img_bin)
    coor_xy= cv2.mean(indices)
    [x,y,z,count]=[0,0,0,0]

    for [[c,r]] in indices:
        aux_x = points[r,c][0]
        aux_y = points[r,c][1]
        aux_z = points[r,c][2]
        if math.isnan(aux_x) or math.isnan(aux_y) or math.isnan(aux_z):
            continue
        [x,y,z,count]=[x+aux_x, y+aux_y, z+aux_z, count+1]

    if count>0:
      x = x/count
      y = y/count
      z = z/count
    else:
      x = 0
      y = 0
      z = 0
    return [coor_xy[0],coor_xy[1],x,y,z]

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

