#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2022-1
# PRACTICE 1 - INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to get an inflated map and a cost map given
# an inflation radius and a cost radius.
#

importar  rospy
importar  numpy
de  nav_msgs . msg  import  OccupancyGrid
de  nav_msgs . srv  importar  GetMap
de  nav_msgs . srv  importar  GetMapResponse
de  nav_msgs . srv  importar  GetMapRequest

NOMBRE  =  "INFANTE_GONZALEZ_HECTOR_SAID"

def  get_inflated_map ( static_map , inflation_cells ):
    print ( "Inflar mapa por"  +  str ( inflacion_celdas ) +  "celdas" )
    inflado  =  numpy . copiar ( mapa_estático )
    [ alto , ancho ] =  mapa_estático . forma
    #
    # HACER:
    # Escribe el código necesario para inflar los obstáculos en el mapa un radio
    # dado por 'inflacion_celdas' (expresado en número de celdas)
    # El mapa se da en 'static_map' como una matriz numérica bidimensional.
    # Considere como celdas ocupadas todas las celdas con un valor de ocupación superior a 50
    #

    para  i  en  rango ( altura ):
    #recorrido desde el punto 0, hasta el punto final del mapa
	para  j  en  rango ( ancho ):
    #la variable "width" representa el ancho de nuestro mapa estamos haciendo un recorrido por el ancho del mapa.
		si  mapa_estático [ i , j ] == 100 :
    #En esta condicacional, se analiza si la celda se encuentra ocupada, el valor 100 dice que esta ocupada, en el caso contrario existiria un valor distinto de 100
			para  k1  en el  rango ( i - celdas_inflación , i + celdas_inflación  +  1 ):
    #Estos dos for, estan definiendo el area que estara ocupada, debido a la inflacion del mapa.
				para  k2  en el  rango ( j - celdas_inflación , j + celdas_inflación  +  1 ):
					inflado [ k1 , k2 ] = static_map [ i , j ]
    volver  inflado

def  get_cost_map ( mapa_estático , radio_costo ):
    si  cost_radius  >  20 :
        cost_radius  =  20
    imprimir  "Calculando el mapa de costos con"  + str ( cost_radius ) +  "celdas"
    mapa_costo  =  numpy . copiar ( mapa_estático )
    [ alto , ancho ] =  mapa_estático . forma
    #
    # HACER:
    # Escriba el código necesario para calcular un mapa de costos para el mapa dado.
    # Para calcular el costo, considere como ejemplo el siguiente mapa:    
    # [[0 0 0 0 0 0]
    # [0 X 0 0 0 0]
    # [0 XX 0 0 0]
    # [0 XX 0 0 0]
    # [0 X 0 0 0 0]
    # [0 0 0 X 0 0]]
    # las celdas ocupadas 'X' tienen un valor de 100 y las celdas libres tienen un valor de 0.
    # El costo es un número entero que indica qué tan cerca están las celdas y los obstáculos:
    # [[3 3 3 2 2 1]
    # [3 X 3 3 2 1]
    # [3 XX 3 2 1]
    # [3 XX 3 2 2]
    # [3 X 3 3 3 2]
    # [3 3 3 X 3 2]]
    # Cost_radius indica el número de celdas alrededor de obstáculos con costos superiores a cero.
    para  i  en  rango ( altura ):
    #este for realiza un recorrido desde el punto cero hasta la altura máxima del mapa
	para  j  en  rango ( ancho ):
		si  mapa_estático [ i , j ] == 100 :
			para  k1  en el  rango ( - cost_radius , cost_radius  +  1 ):
				para  k2  en el  rango ( - cost_radius , cost_radius  +  1 ):
					costo =  costo_radio - max ( abs ( k1 ), abs ( k2 ))
					mapafinal =  mapa_costo [ i + k1 , j + k2 ]
					mapa_costo [ i  +  k1 , j  +  k2 ] = max ( costo , mapafinal )
    imprimir  mapa_de_costos
    retorno  cost_map

def  callback_inflated_map ( req ):
     mapa_inflado global
    retorno  GetMapResponse ( mapa = inflated_map )

def  callback_cost_map ( req ):
     mapa_de_costo global
    return  GetMapResponse ( map = cost_map )
    
def  principal ():
    mundial  cost_map , inflated_map
    imprimir  "PRÁCTICA 01 -"  +  NOMBRE
    rosado . init_node ( "práctica01" )
    rosado . esperar_servicio ( '/ mapa_estático' )
    pub_map   =  rospy . Editor ( "/ inflated_map " , OccupancyGrid , queue_size = 10 )
    grid_map  =  rospy . ServiceProxy ( "/ mapa_estático" , GetMap ) (). mapa
    map_info  =  grid_map . info
    ancho , alto , res  =  map_info . ancho , map_info . altura , map_info . resolución
    grid_map  =  numpy . remodelar ( numpy . asarray ( grid_map . data , dtype = 'int' ), ( alto , ancho ))
    rosado . Servicio ( '/ inflated_map ' , GetMap , callback_inflated_map )
    rosado . Servicio ( '/ cost_map'     , GetMap , callback_cost_map )
    bucle  =  rosado . Tasa ( 1 )
    
    cost_radius       =  0.1
    radio_inflación  =  0.1
    aunque  no  rosado . is_shutdown ():
        si  rosado . has_param ( "/ path_planning / cost_radius" ):
            new_cost_radius  =  rospy . get_param ( "/ path_planning / cost_radius" )
        si  rosado . has_param ( "/ path_planning / inflacion_radius" ):
            nuevo_radio_inflación  =  rospy . get_param ( "/ path_planning / inflacion_radius" )
        if  new_cost_radius  ! =  cost_radius :
            cost_radius    =  new_cost_radius
            cost_map_data  =  get_cost_map ( grid_map , int ( cost_radius / res ))
            cost_map_data  =  numpy . ravel ( numpy . reshape ( cost_map_data , ( ancho * alto , 1 )))
            cost_map       =  OccupancyGrid ( info = map_info , data = cost_map_data )    
        Si  new_inflation_radius  =!  inflation_radius :
            inflation_radius   =  new_inflation_radius
            inflated_map_data  =  get_inflated_map ( grid_map , int ( inflation_radius / res ))
            inflated_map_data  =  numpy . enmarañamiento ( numpy . reshape ( inflated_map_data , ( anchura * altura , 1 )))
            inflated_map       =  OccupancyGrid ( info = map_info , datos = inflated_map_data )
            pub_map . publicar ( callback_inflated_map ( GetMapRequest ()). map )
        bucle . dormir ()

if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass