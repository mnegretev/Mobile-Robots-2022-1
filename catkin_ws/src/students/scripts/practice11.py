#! / usr / bin / env python
#
# ROBOTS MÓVILES AUTÓNOMOS - UNAM, FI, 2022-1
# PRÁCTICA 11 - ENTRENAMIENTO DE UNA RED NEURAL
#
# Instrucciones:
# Complete el código para entrenar una red neuronal para
# reconocimiento de dígitos escritos a mano.
#
importar  cv2
importar  sys
importar al  azar
importar  numpy
importar  rospy
importar  rospkg

NOMBRE  =  "Gutierrez_Alcibar"

clase  NeuralNetwork ( objeto ):
    def  __init__ ( self , capas , pesos = Ninguno , sesgos = Ninguno ):
        #
        # La lista 'capas' indica el número de neuronas en cada capa.
        # Recuerde que la primera capa indica la dimensión de las entradas y así,
        # no hay vector de sesgo para la primera capa.
        # Para esta práctica, 'capas' debe ser algo como [784, n2, n3, ..., nl, 10]
        # Todos los pesos y sesgos se inicializan con valores aleatorios. En cada capa tenemos una matriz
        # de pesos donde la fila j contiene todos los pesos de la j-ésima neurona en esa capa. Para este ejemplo,
        # la primera matriz debe ser del orden n2 x 784 y la última matriz debe ser 10 x nl.
        #
        yo . num_layers   =  len ( capas )
        yo . layer_sizes  =  capas
        yo . sesgos  = [ numpy . al azar . randn ( y , 1 ) para  y  en  capas [ 1 :]] si  sesgos  ==  Ninguno  más  sesgos
        yo . pesos = [ numpy . al azar . randn ( y , x ) para  x , y  en  zip ( capas [: - 1 ], capas [ 1 :])] si  pesos == Ninguno  más  pesos
        
    def  feedforward ( self , x ):
        #
        # Esta función obtiene la salida de la red cuando la entrada es 'x'.
        #
        para  i  en  rango ( len ( auto . sesgos )):
            z  =  numpy . dot ( auto . pesos [ i ], x ) +  auto . sesgos [ i ]
            x  =  1.0  / ( 1.0  +  numpy . exp ( - z ))   # la salida de la capa actual es la entrada de la siguiente
        volver  x

    def  feedforward_verbose ( self , x ):
        #
        # HACER:
        # Escriba una función similar a 'feedforward' pero en lugar de devolver solo la capa de salida,
        # devuelve una lista que contiene la salida de cada capa, desde la entrada hasta la salida.
        # Incluya la entrada x como la primera salida.
        #
        y  = []
	y . añadir ( x )
	para  i  en el  rango ( len ( self . layer_sizes ) - 1 ):
		u  =  numpy . dot ( auto . pesos [ i ], x ) + auto . sesgos [ i ]
		x  =  1.0 / ( 1.0  +  número . exp ( - u ))
		y . añadir ( x )
        volver  y

    def  backpropagate ( self , x , yt ):
        y  =  self . feedforward_verbose ( x )
        nabla_b  = [ numpy . ceros ( b . forma ) para  b  en  sí mismo . sesgos ]
        nabla_w  = [ numpy . ceros ( w . forma ) para  w  en  sí mismo . pesos ]
        # HACER:
        # Devuelve una tupla [nabla_w, nabla_b] que contiene el gradiente de la función de costo C con respecto a
        # cada peso y sesgo de toda la red. El gradiente se calcula asumiendo solo un entrenamiento
        # Se da un ejemplo: la entrada 'x' y la etiqueta correspondiente 'yt'.
        # nabla_w y nabla_b deben tener las mismas dimensiones que las correspondientes
        # self.weights y self.biases
        # Puede calcular el gradiente siguiendo estos pasos:
        #
        # Calcule el delta para la capa de salida L: delta = (yL-yt) * yL * (1-yL)
        # nabla_b de la capa de salida = delta      
        # nabla_w de la capa de salida = delta * yLpT donde yLpT es la transposición del vector de salida de la capa L-1
        # PARA todas las capas 'l' desde L-1 hasta la capa de entrada: 
        # delta = (WT * delta) * yl * (1 - yl)
        # donde 'WT' es la transposición de la matriz de pesos de la capa l + 1 e 'yl' es la salida de la capa l
        # nabla_b [-l] = delta
        # nabla_w [-l] = delta * ylpT donde ylpT es la transposición del vector de salida de la capa l-1
        #        
	delta  = ( y [ - 1 ] - yt ) * y [ - 1 ] * ( 1 - y [ - 1 ])
	nabla_b [ - 1 ] =  delta
	nabla_w [ - 1 ] =  delta * numpy . transponer ( y [ - 2 ])
	para  i  en el  rango ( 2 , len ( self . layer_sizes )):
		delta  =  numpy . dot ( numpy . transpuesta ( auto . pesos [ - i + 1 ]), delta ) * y [ - i ] * ( 1 - y [ - i ])
		nabla_b [ - i ] =  delta
		nabla_w [ - i ] =  delta * numpy . transponer ( y [ - i - 1 ])
        volver  nabla_w , nabla_b

    def  update_with_batch ( self , batch , eta ):
        #
        # Esta función ejecuta el descenso de gradiente para el subconjunto de ejemplos
        # dado por 'lote' con tasa de aprendizaje 'eta'
        # 'lote' es una lista de ejemplos de entrenamiento [(x, y), ..., (x, y)]
        #
        nabla_b  = [ numpy . ceros ( b . forma ) para  b  en  sí mismo . sesgos ]
        nabla_w  = [ numpy . ceros ( w . forma ) para  w  en  sí mismo . pesos ]
        M  =  len ( lote )
        para  x , y  en  lote :
            si  rosado . is_shutdown ():
                rotura
            delta_nabla_w , delta_nabla_b  =  self . retropropagar ( x , y )
            nabla_w  = [ nw + dnw  para  nw , dnw  en  zip ( nabla_w , delta_nabla_w )]
            nabla_b  = [ nb + dnb  para  nb , dnb  en  zip ( nabla_b , delta_nabla_b )]
        yo . pesos  = [ w - eta * nw / M  para  w , nw  en  zip ( auto . pesos , nabla_w )]
        yo . sesgos   = [ b - eta * nb / M  para  b , nb  en  zip ( self . sesgos , nabla_b )]
        volver  nabla_w , nabla_b

    def  get_gradient_mag ( self , nabla_w , nabla_b ):
        mag_w  =  sum ([ numpy . sum ( n ) para  n  en [ nw * nw  para  nw  en  nabla_w ]])
        mag_b  =  sum ([ numpy . sum ( b ) para  b  en [ nb * nb  para  nb  en  nabla_b ]])
        return  mag_w  +  mag_b

    def  train_by_SGD ( auto , training_data , épocas , batch_size , eta ):
        para  j  en  rango ( épocas ):
            al azar . shuffle ( datos_de_entrenamiento )
            batches  = [ training_data [ k : k + batch_size ] for  k  in  range ( 0 , len ( training_data ), batch_size )]
            para  lotes  en  lotes :
                si  rosado . is_shutdown ():
                    regreso
                nabla_w , nabla_b  =  self . update_with_batch ( lote , eta )
                sys . stdout . write ( " \ r Magnitud de gradiente:% f"  % ( self . get_gradient_mag ( nabla_w , nabla_b )))
                sys . stdout . rubor ()
            print ( "Época:"  +  str ( j ))
    #
    ### FIN DE CLASES
    #


def  load_dataset ( carpeta ):
    imprimir  "Cargando conjunto de datos desde"  +  carpeta
    si  no  carpeta . termina con ( "/" ):
        carpeta  + =  "/"
    training_dataset , training_labels , testing_dataset , testing_labels  = [], [], [], []
    para  i  en el  rango ( 10 ):
        f_data  = [ ord ( c ) / 255.0  para  c  en  open ( carpeta  +  "datos"  +  str ( i ), "rb" ). leer ( 784000 )]
        imágenes  = [ numpy . asarray ( f_data [ 784 * j : 784 * ( j + 1 )]). remodelar ([ 784 , 1 ]) para  j  en el  rango ( 1000 )]
        etiqueta   =  numpy . asarray ([ 1  si  i  ==  j  más  0  para  j  en el  rango ( 10 )]). remodelar ([ 10 , 1 ])
        training_dataset  + =  images [ 0 : len ( imágenes ) // 2 ]
        training_labels   + = [ etiqueta  para  j  en  rango ( len ( imágenes ) // 2 )]
        testing_dataset   + =  images [ len ( imágenes ) // 2 : len ( imágenes )]
        testing_labels    + = [ etiqueta  para  j  en  rango ( len ( imágenes ) // 2 )]
    return  zip ( training_dataset , training_labels ), zip ( testing_dataset , testing_labels )

def  principal ():
    print ( "PRÁCTICA 11 -"  +  NOMBRE )
    rosado . init_node ( "práctica11" )
    rospack  =  rospkg . RosPack ()
    carpeta_conjunto_datos  =  rospack . get_path ( "config_files" ) +  "/ handwritten_digits /"
    épocas         =  3
    tamaño_lote     =  10
    tasa_de_aprendizaje  =  3.0
    
    si  rosado . has_param ( "~ épocas" ):
        épocas  =  rosado . get_param ( "~ épocas" )
    si  rosado . has_param ( "~ tamaño_de_lote" ):
        tamaño_lote  =  rospy . get_param ( "~ tamaño_de_lote" )
    si  rosado . has_param ( "~ tasa_de_aprendizaje " ):
        learning_rate  =  Rospy . get_param ( "~ tasa_de_aprendizaje " )

    training_dataset , testing_dataset  =  load_dataset ( carpeta_datos )
    
    prueba :
        datos_ guardados  =  numpy . cargar ( carpeta_datos + "network.npz" , allow_pickle = True )
        capas  = [ datos_ guardados [ 'w' ] [ 0 ]. forma [ 1 ]] + [ b . forma [ 0 ] para  b  en  datos_ guardados [ 'b' ]]
        nn  =  NeuralNetwork ( capas , pesos = datos_ guardados [ 'w' ], sesgos = datos_ guardados [ 'b' ])
        print ( "Cargando datos de un modelo previamente entrenado con capas"  +  str ( capas ))
    excepto :
        nn  =  Red neuronal ([ 784 , 30 , 10 ])
        aprobar
    
    nn . train_by_SGD ( training_dataset , épocas , batch_size , learning_rate )
    numpy . savez ( carpeta_datos  +  "red" , w = nn . pesos , b = nn . sesgos )
    
    imprimir ( " \ n Presione la tecla para probar la red o ESC para salir ..." )
    numpy . set_printoptions ( formatter = { 'float_kind' : "{: .3f}" . formato })
    cmd  =  cv2 . waitKey ( 0 )
    mientras que  cmd  ! =  27  y  no  rosado . is_shutdown ():
        img , label  =  testing_dataset [ numpy . al azar . randint ( 0 , 4999 )]
        y  =  nn . feedforward ( img ). transponer ()
        print ( " \ n Salida del perceptrón:"  +  str ( y ))
        print ( "Salida esperada:"    +  str ( label . transpose ()))
        print ( "Dígito reconocido:"    +  str ( numpy . argmax ( y )))
        cv2 . imshow ( "Dígito" , numpy . reshape ( numpy . asarray ( img , dtype = "float32" ), ( 28 , 28 , 1 )))
        cmd  =  cv2 . waitKey ( 0 )
    

if  __name__  ==  '__main__' :
    principal ()
