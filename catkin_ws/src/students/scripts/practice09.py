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

NAME = "GALARZA_MARTINEZ_ABEL"

def segment_by_color(img_bgr, points):
    #
    # TODO:
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the color range of the ball.
    #   Check online documentation for cv2.inRange
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    #
    
    img_hsv=cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV) #Cambiar de RGB a HSV--> funcion  cv2.cvtColor
    img_bin=cv2.inRange(img_hsv, numpy.array([25,200,50]),numpy.array([35,255,255])) #para determinar el color que ets ane el rango del circulo
   
    idn= cv2.findNonZero(img_bin) #calculo de centroide de todos los pixeles para definir la posicion de la bola
    cds= cv2.mean(ind)
    [x,y,z,cont]=[0,0,0,0]
    
    
    for[[c,r]] in ind:
	x1=points[r,c][0]
	y1=points[r,c][1]
	z1=points[r,c][2]
	if math.isnan(x1) or math.isnan(y1) or math.isnan(z1):
		continue
	[x,y,z,cont] = [x1+x, y1+y, z1+z,cont+1]
    
    if cont>0:
    x = x/cont
	y = y/cont
	z = z/cont
    else: 
	x,y,z=0,0,0
    

    return [0,0,0,0,0]

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

