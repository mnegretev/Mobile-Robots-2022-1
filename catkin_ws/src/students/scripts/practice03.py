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
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from custom_msgs.srv import SmoothPath
from custom_msgs.srv import SmoothPathResponse

NAME = "Manzo_Soto_Jorge_Luis"

msg_smooth_path = Path()

def smooth_path(Q, alpha, beta):
    print("Smoothing path with params: " + str([alpha,beta]))
    #
    # TODO:
    # Write the code to smooth the path Q, using the gradient descend algorithm,
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
    # Despues de considerar que nuestro primer y ultimo termino van a ser cero se progaman los siguientes comandos 
    # Primero indicamos que nuestro primer termino de nabla sera cero de la siguiente manera.                
    nabla[0]=0
    # Posteriormente indicamos que nuestro ultimo termino tambien sera cero, como se muestra a continuacion.
    nabla[len(nabla)-1]=0
    # Inicializamos nuestra variable steps en cero para comenzar con la operacion.
    steps=0
    # En la siguiente condicional que si la magnitud de nuestra variable nabla sea mayor que tol, que se trata de un 
    # valor cercano en cero y que nuestra variable steps sea menor que 1000, se comienza con la operacion.
    while numpy.linalg.norm(nabla) > tol and steps <100000:
    # Con este for calculamos el gradiente que necesitamos para marcar la ruta, aqui indicamos que nuestra variable i
    # comenzara desde el punto 1, hasta el ultimo punto de nuestra nabla, para esto calculamos nabla.
      for i in range (1, len(nabla)-1): 
    # Para el calculo de nabla aplicamos lo calculado en clase, lo cual es lo siguiente.
    # nabla=alpha por el iesimo punto menos el correspondiente punto de la ruta original mas beta por dos veces el 
    # iesimo punto, menos el punto posterior, menos el punto anterior. 
        nabla[i]=beta*(P[i]-Q[i])+alpha*(2*P[i]-P[i-1]-P[i+1])
    # Posteriormente, indicamos que el conjunto de nuestros puntos se van a mover en sentido contrario a la epsilon por 
    # nuestro gradiente que lleva por nombre nabla
      P=P-epsilon*nabla
    # Por ultimo, le sumamos una unidad a nuestra variable step 
      steps+=1
    return P

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
    
