#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

#Receive the message with that name (msg)
def callback(msg):
    rospy.loginfo("I heard %s", msg.data)

if __name__ == '__main__':
    rospy.init_node("listener") #Inicializar el nodo

    #Ya no vamos a publicar sino escuchar
    rospy.Subscriber("chatter", String, callback) #topic, type of message, interrupt

    #No se pondria nada en el while, por eso se usa el spin.
    #Sleep waits, Spin stays idle
    rospy.spin() 