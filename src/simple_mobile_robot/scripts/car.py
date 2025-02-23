#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions
from visualization_msgs.msg import Marker

# Setup parameters, variables and callback functions
#Declare msg
sun = Marker()
def init_sun () : 
    # Header
    sun.header.frame_id = "run" # nombre del fixed frame
    sun.header.stamp = rospy.Time.now() 

    # Set shape 
    sun.id = 0
    sun.type = 1
    # Add Marker 
    sun.action = 0 
    
    # Set pos Marker
    sun.pose.position.x = 0.0
    sun.pose.position.y = 0.0
    sun.pose.position.z = 0.0
    sun.pose.orientation.x = 0.0
    sun.pose.orientation.y = 0.0
    sun.pose.orientation.z = 0.0
    sun.pose.orientation.w = 1.0

    # Set the scale of the marker
    sun.scale.x = 2.0
    sun.scale.y = 2.0
    sun.scale.z = 2.0 

    # Set the color
    sun.color.r = 1.0
    sun.color.g = 1.0
    sun.color.b = 0.0
    sun.color.a = 1.0

    # Set Duration
    sun.lifetime = rospy.Duration (0)

    # Stop Condition 
def stop() : 
        print ("Stopping")

if __name__ == "__main__":
    rospy.init_node("diferential_robot")
    loop_rate = rospy.Rate(10) #10 hz
    rospy.on_shutdown(stop)
    print("The sun is ready")

    # Setup the message 
    init_sun()

    # Set publishers and subscribers 
    pub_sun = rospy.Publisher('/sun' , Marker , queue_size = 1 )

    try:
        while not rospy.is_shutdown() :
            sun.header.stamp = rospy.Time.now()
            pub_sun.publish(sun)
            loop_rate.sleep()
    except rospy.ROSInterruptException : 
        pass 
