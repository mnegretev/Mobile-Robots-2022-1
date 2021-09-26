#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 2 - PATH PLANNING BY A-STAR
#
# Instructions:
# Write the code necessary to plan a path using an
# occupancy grid and the A* algorithm
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "REZA CHAVARRIA"

msg_path = Path()

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write the A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return a set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # indicating the indices (cell coordinates) of the path cells.
    # If path cannot be found, return an empty tuple []
    #
    #COLA CON PRIORIRDAD
    open_list=[]
    g_values= numpy.full(grid_map.shape,float("inf"))
    f_values= numpy.full(grid_map.shape,float("inf"))
    #Nodo previo
    previous= numpy.full((grid_map.shape[0],grid_map.shape[0],2),-1)
    #listas abiertas y cerradas con binarios
    in_closed_l=numpy.full(grid_map.shape,False)
    in_open_l=numpy.full(grid_map.shape,False)
    
    #Valores de nodos adjacentes (forma 8)
    adjacents   = [[1,0],[0,1],[-1,0],[0,-1], [1,1], [-1,1], [-1,-1],[1,-1]]
    heapq.heappush(open_list,(0,[start_r,start_c]))
    #Inicializacion de valores y revision de valores del nodo inicial
    in_open_l[start_r,start_c]=True
    g_values[start_r,start_c]=0
    f_values[start_r,start_c]=0
    [row,col]=[start_r,start_c]

    while len(open_list)>0 and [row,col]!=[goal_r,goal_c]:
      #Size del mapa
      [row,col]=heapq.heappop(open_list)[1]
      #Agregar a la lista cerrada
      in_closed_l[row,col]=True
      #Obtener los nodos adyacentes
      adjacents_nodes = [[row+i, col+j] for [i,j] in adjacents]

      for [r,c] in adjacents_nodes:
        #Si los nodos adyacentes no valen 0 o ya estan en la lista cerrada
        #Terminar la iteracion
        if grid_map[r,c]!=0 or in_closed_l[r,c]:
          continue

        #Calculo de valor de G con los valores de costo y la distancia entre nodos
        g=g_values[row,col]+ math.sqrt((row-r)**2+(col-c)**2)+cost_map[row,col]
        #Distancia entre el nodo actual y el nodo de meta
        h=math.sqrt((goal_r-r)**2 + (goal_c-c)**2)
        #Valor de revision para ruta estrella
        f=g+h
        #Asignacion de valores en las tablas
        if g <g_values[r,c]:
          g_values[r,c]=g
          f_values[r,c]=f
          #Asignar nodo previo
          previous[r,c]=[row,col]

        #SI no esta en la lista abierta el nodo
        if not in_open_l[r,c]:
          #Agregar a la lista abierta
          heapq.heappush(open_list,(f_values[r,c],[r,c]))
          in_open_l[r,c]=True

    #Si el valor no es igual al punto final
    if [row,col]!= [goal_r,goal_c]:
      print("Cannot calculate path")
      return []
    print("Path complete")

    path = []
    #Agregar a la ruta los nodos previos
    while [previous[row,col][0],previous[row,col][1]]!=[-1,-1]:
      path.insert(0,[row,col])
      #Obtener el renglon y columna del nodo previo
      [row,col]=previous[row,col]
    return path



def get_maps():
    print("Getting inflated and cost maps...")
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    try:
        static_map = clt_static_map().map
    except:
        print("Cannot get static map. Terminating program. ")
        exit()
    try:
        inflated_map = clt_inflated().map
        cost_map     = clt_cost_map().map
        print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
        print("Using cost map with "     +str(len(cost_map.data))     + " cells.")
    except:
        inflated_map = static_map
        cost_map     = static_map
        print("Cannot get augmented maps. Using static map instead.")
    inflated_map = numpy.reshape(numpy.asarray(inflated_map.data), (static_map.info.height, static_map.info.width))
    cost_map     = numpy.reshape(numpy.asarray(cost_map.data)    , (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map, cost_map]

def callback_a_star(req):
    [s_map, inflated_map, cost_map] = get_maps()
    res = s_map.info.resolution
    [sx, sy] = [req.start.pose.position.x, req.start.pose.position.y]
    [gx, gy] = [req.goal .pose.position.x, req.goal .pose.position.y]
    [zx, zy] = [s_map.info.origin.position.x, s_map.info.origin.position.y]
    print("Calculating path by A* from " + str([sx, sy])+" to "+str([gx, gy]))
    path = a_star(int((sy-zy)/res), int((sx-zx)/res), int((gy-zy)/res), int((gx-zx)/res), inflated_map, cost_map)
    msg_path.poses = []
    for [r,c] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=(c*res + zx), y=(r*res + zy)))))
    return GetPlanResponse(msg_path)

def main():
    print "PRACTICE 02 - " + NAME
    rospy.init_node("practice02")
    rospy.wait_for_service('/static_map')
    rospy.Service('/path_planning/a_star_search'  , GetPlan, callback_a_star)
    pub_path = rospy.Publisher('/path_planning/a_star_path', Path, queue_size=10)
    loop = rospy.Rate(2)
    msg_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
