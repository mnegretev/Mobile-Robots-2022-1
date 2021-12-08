#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 1 - INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to get an inflated map and a cost map given
# an inflation radius and a cost radius.
#

import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "Equipo1"

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # TODO:
    # Write the code necessary to inflate the obstacles in the map a radius
    # given by 'inflation_cells' (expressed in number of cells)
    # Map is given in 'static_map' as a bidimensional numpy array.
    # Consider as occupied cells all cells with an occupation value greater than 50
    #

    #
    #Antes que nada tenemos que leer el mapa completo y buscar las celdas que ya
    #esten ocupadas. Dado que es un mapa con coordenadas bidimensionales,
    #utilizaremos dos arreglos 'for' con las variables 'i', 'j' para recorrerlo.
    #El rango de los arreglos 'for' estan limitados por el tamano y ancho del
    #mapa, los cuales definimos con las variables 'height' y 'width'.
    #
    for i in range(height):
        for j in range(width):

    #
    #Mientras recorremos el mapa, vamos leyendo el valor de las celdas para ver si
    #estan ocupadas o no. Si es el primer caso, la celda tiene un valor igual a
    #100. Por ello utilizamos el siguiente 'if'.
    #
            if static_map[i,j] == 100:

    #
    #Una vez que encontramos una celda ocupada, inflamos las celdas aledanas con
    #la cantidad especificada en la variable 'inflated_cells'. Para esto utilizamos
    #nuevamente un arreglo for para marcar las celdas infladas en nuestro mapa
    #'inflated' con tal de no modificar el mapa original.
    #
                for k1 in range(i-inflation_cells,i+inflation_cells+1):
                    for k2 in range(j-inflation_cells, j+inflation_cells+1):
                        inflated[k1,k2] = static_map[i,j]

    return inflated

def get_cost_map(static_map, cost_radius):
    if cost_radius > 20:
        cost_radius = 20
    print "Calculating cost map with " +str(cost_radius) + " cells"
    cost_map = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # TODO:
    # Write the code necessary to calculate a cost map for the given map.
    # To calculate cost, consider as example the following map:    
    # [[ 0 0 0 0 0 0]
    #  [ 0 X 0 0 0 0]
    #  [ 0 X X 0 0 0]
    #  [ 0 X X 0 0 0]
    #  [ 0 X 0 0 0 0]
    #  [ 0 0 0 X 0 0]]
    # Where occupied cells 'X' have a value of 100 and free cells have a value of 0.
    # Cost is an integer indicating how near cells and obstacles are:
    # [[ 3 3 3 2 2 1]
    #  [ 3 X 3 3 2 1]
    #  [ 3 X X 3 2 1]
    #  [ 3 X X 3 2 2]
    #  [ 3 X 3 3 3 2]
    #  [ 3 3 3 X 3 2]]
    # Cost_radius indicate the number of cells around obstacles with costs greater than zero.
    
    #
    #Para resolver esta parte, vamos a hacer lo mismo que en el primer ejercicio de esta 
    #practica. Nuevamente recorremos el mapa completo con un arreglo for y buscamos celdas
    #que esten ocupadas con un valor de 100. Cuando encontremos una tambien vamos a rellenar
    #las celdas aledanas, solo que ahora con un valor numerico.
    #

    for i in range(height):
        for j in range(width):
            if static_map[i,j] == 100:
                for k1 in range(-cost_radius, cost_radius+1):
                    for k2 in range(-cost_radius,cost_radius+1):

    #
    #Ahora tenemos que agregar los valores numericos alrededor de las celdas ocupadas, en
    #donde las celdas mas cercanas a la que esta ocupada tienen un valor mas grande, el cual
    #decrece conforme se aleja de esta primera (con base en los parametros 'cost_radius'
    #
        
                        cost = cost_radius - max(abs(k1), abs(k2))
                        mapa_apoyo = cost_map[i+k1, j+k2]
                        cost_map[i+k1, j+k2] = max(cost, mapa_apoyo)

    return cost_map

def callback_inflated_map(req):
    global inflated_map
    return GetMapResponse(map=inflated_map)

def callback_cost_map(req):
    global cost_map
    return GetMapResponse(map=cost_map)
    
def main():
    global cost_map, inflated_map
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    rospy.wait_for_service('/static_map')
    pub_map  = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    grid_map = rospy.ServiceProxy("/static_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    rospy.Service('/inflated_map', GetMap, callback_inflated_map)
    rospy.Service('/cost_map'    , GetMap, callback_cost_map)
    loop = rospy.Rate(1)
    
    cost_radius      = 0.1
    inflation_radius = 0.1
    while not rospy.is_shutdown():
        if rospy.has_param("/path_planning/cost_radius"):
            new_cost_radius = rospy.get_param("/path_planning/cost_radius")
        if rospy.has_param("/path_planning/inflation_radius"):
            new_inflation_radius = rospy.get_param("/path_planning/inflation_radius")
        if new_cost_radius != cost_radius:
            cost_radius   = new_cost_radius
            cost_map_data = get_cost_map(grid_map, int(cost_radius/res))
            cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (width*height, 1)))
            cost_map      = OccupancyGrid(info=map_info, data=cost_map_data)    
        if new_inflation_radius != inflation_radius:
            inflation_radius  = new_inflation_radius
            inflated_map_data = get_inflated_map(grid_map, int(inflation_radius/res))
            inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (width*height, 1)))
            inflated_map      = OccupancyGrid(info=map_info, data=inflated_map_data)
            pub_map.publish(callback_inflated_map(GetMapRequest()).map)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
