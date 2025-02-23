#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32

#Variable for the sine wave. This will be sent. Only variable that can be modified
amplitude= 1.0

#Variables to be sent
signal= 0.0
time = 0.0

if __name__ == '__main__': #Aqui empieza el programa
    rospy.init_node('signal_generator') #Inicializamos el nodo signal_generator

    #Create channels /signal and /time
    signal_pub = rospy.Publisher('signal', Float32, queue_size=10) 
    time_pub = rospy.Publisher('time', Float32, queue_size=10) 
    amp_pub = rospy.Publisher('amplitude', Float32, queue_size=10)

    #10 Hz
    rate = rospy.Rate(10)


    while not rospy.is_shutdown(): #void loop
        
        #Sine wave generator
        time= time + 0.1
        signal= amplitude * np.sin(time)

        #Publish signal and time
        signal_pub.publish(signal)
        amp_pub.publish(amplitude)
        time_pub.publish(time) 

        rate.sleep()

