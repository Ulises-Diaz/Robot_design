#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32

#Variables received from "signal_generator"
signal_received= 0.0
time_received= 0.0
amplitude_received= 0.0

#Variables used to process sine wave
proc_signal= 0.0
proc_amplitude= 0.0
proc_offset= 0.0

#Constants used to process sine wave
proc_phase= 0.5

#Receive the message with the signal
def sig_callback(signal):
    global signal_received
    signal_received= signal.data

    #Debug message
    #rospy.loginfo("The signal is %s", signal_received)

#Receive the message with the time
def time_callback(time):
    global time_received
    time_received= time.data

    #Debug message
    #rospy.loginfo("The time is %s", time_received)

    #Receive the message with the time
def amp_callback(amplitude):
    global amplitude_received
    amplitude_received= amplitude.data

    #Debug message
    #rospy.loginfo("The amplitude is %s", amplitude_received)

if __name__ == '__main__':
    rospy.init_node("process") #Inicializar el nodo process

    #Subscribe data from /signal and/time
    rospy.Subscriber("/signal", Float32, sig_callback) 
    rospy.Subscriber("/time", Float32, time_callback)
    rospy.Subscriber("/amplitude", Float32, amp_callback)

    #Create channel for processed signal
    proc_pub = rospy.Publisher('proc_signal', Float32, queue_size=10)

    #10 Hz
    rate= rospy.Rate(10)

    while not rospy.is_shutdown():

        #Process the sine wave automatically
        proc_amplitude= amplitude_received/2
        proc_offset= amplitude_received/2

        #Processed signal using the variables
        proc_signal= proc_amplitude*np.sin(time_received - proc_phase) + proc_offset

        #Publish the processed signal
        proc_pub.publish(proc_signal)

        rate.sleep() 