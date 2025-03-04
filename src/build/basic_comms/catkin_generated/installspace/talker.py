#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__': #Aqui empieza el programa
    rospy.init_node('talker') #Inicializamos el nodo

    pub = rospy.Publisher('chatter', String, queue_size=10) #Crea un canal
    rate = rospy.Rate(10)

    #Agarraremos el parametro. (nombre del param, mensaje en caso de no encontrarlo)
    #("Message") Local param
    #("/Message") Global param
    #("~Message") Private param
    parameter= rospy.get_param("/Message", "No param found")



    msg = String()

    while not rospy.is_shutdown(): #void loop
        #Se usa el data del documentation. El point seria msg.x msg.y etc.
        msg.data = parameter 

        pub.publish(msg) #Publish = mandar la informacion

        #Hay que eliminar linea 161 hasta 164 del CMakeLists
        #Cambiar el nombre al nombre del programa
        rate.sleep()

        #Hay que compilar con catkin_make en catkin *terminal*