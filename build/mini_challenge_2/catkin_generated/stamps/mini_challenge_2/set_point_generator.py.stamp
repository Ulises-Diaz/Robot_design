#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import numpy as np

def signal_generator():
    rospy.init_node('signal_generator')
    signal_pub = rospy.Publisher('/set_point', Float32, queue_size=10)
    #time_pub = rospy.Publisher('/time', Float32, queue_size=10)
    rate = rospy.Rate(10)  

    t = 0.0
    
    while not rospy.is_shutdown():
        y = np.sin(t)
        signal_pub.publish(y)
        #time_pub.publish(t)
        rospy.loginfo("Signal: %f, Time: %f", y, t)
        t += 0.1
        rate.sleep()

if __name__ == '__main__':
    try:
        signal_generator()
    except rospy.ROSInterruptException:
        pass