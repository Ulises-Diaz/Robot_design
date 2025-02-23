#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions
from visualization_msgs.msg import Marker

# Setup parameters, variables and callback functions
#Declare msg
caster_wheel = Marker()
wheel_r = Marker()
wheel_l = Marker()
chassis = Marker()
world = Marker ()

def init_world () : 
    # Header
    world.header.frame_id = "world" # nombre del fixed frame
    world.header.stamp = rospy.Time.now() 

    # Set shape 
    world.id = 5
    world.type = 2
    # Add Marker 
    world.action = 0 
    
    # Set pos Marker
    world.pose.position.x = 0.0
    world.pose.position.y = 0.0
    world.pose.position.z = 5.0
    world.pose.orientation.x = 0.0
    world.pose.orientation.y = 0.0
    world.pose.orientation.z = 0.0
    world.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    world.scale.x = 0.5
    world.scale.y = 0.5
    world.scale.z = 0.5 

    # Set the color
    world.color.r = 1.0
    world.color.g = 1.0
    world.color.b = 0.0
    world.color.a = 1.0

    # Set Duration
    world.lifetime = rospy.Duration (0)


def init_caster_wheel () : 
    # Header
    caster_wheel.header.frame_id = "caster_wheel" # nombre del fixed frame
    caster_wheel.header.stamp = rospy.Time.now() 

    # Set shape 
    caster_wheel.id = 0
    caster_wheel.type = 2
    # Add Marker 
    caster_wheel.action = 0 
    
    # Set pos Marker
    caster_wheel.pose.position.x = 0.0
    caster_wheel.pose.position.y = 0.0
    caster_wheel.pose.position.z = 0.25
    caster_wheel.pose.orientation.x = 0.0
    caster_wheel.pose.orientation.y = 0.0
    caster_wheel.pose.orientation.z = 0.0
    caster_wheel.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    caster_wheel.scale.x = 0.5
    caster_wheel.scale.y = 0.5
    caster_wheel.scale.z = 0.5 

    # Set the color
    caster_wheel.color.r = 1.0
    caster_wheel.color.g = 1.0
    caster_wheel.color.b = 0.0
    caster_wheel.color.a = 1.0

    # Set Duration
    caster_wheel.lifetime = rospy.Duration (0)

def init_wheel_r() : 
    # Header
    wheel_r.header.frame_id = "wheel_r" # nombre del fixed frame
    wheel_r.header.stamp = rospy.Time.now() 

    # Set shape 
    wheel_r.id = 1
    wheel_r.type = 2
    # Add Marker 
    wheel_r.action = 0 
    
    # Set pos Marker
    wheel_r.pose.position.x = 0.0
    wheel_r.pose.position.y = 1.0
    wheel_r.pose.position.z = 0.4
    wheel_r.pose.orientation.x = 0.0
    wheel_r.pose.orientation.y = 0.0
    wheel_r.pose.orientation.z = 0.0
    wheel_r.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    wheel_r.scale.x = 0.5
    wheel_r.scale.y = 0.5
    wheel_r.scale.z = 0.5 

    # Set the color
    wheel_r.color.r = 1.0
    wheel_r.color.g = 1.0
    wheel_r.color.b = 0.0
    wheel_r.color.a = 1.0

    # Set Duration
    wheel_r.lifetime = rospy.Duration (0)

def init_wheel_l() : 
    # Header
    wheel_l.header.frame_id = "wheel_l" # nombre del fixed frame
    wheel_l.header.stamp = rospy.Time.now() 

    # Set shape 
    wheel_l.id = 2
    wheel_l.type = 2
    # Add Marker 
    wheel_l.action = 0 
    
    # Set pos Marker
    wheel_l.pose.position.x = 3.0
    wheel_l.pose.position.y = 1.0
    wheel_l.pose.position.z = 0.4
    wheel_l.pose.orientation.x = 0.0
    wheel_l.pose.orientation.y = 0.0
    wheel_l.pose.orientation.z = 0.0
    wheel_l.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    wheel_l.scale.x = 0.5
    wheel_l.scale.y = 0.5
    wheel_l.scale.z = 0.5 

    # Set the color
    wheel_l.color.r = 1.0
    wheel_l.color.g = 1.0
    wheel_l.color.b = 0.0
    wheel_l.color.a = 1.0

    # Set Duration
    wheel_l.lifetime = rospy.Duration (0)

def init_chassis() : 
    # Header
    chassis.header.frame_id = "chassis" # nombre del fixed frame
    chassis.header.stamp = rospy.Time.now() 

    # Set shape 
    chassis.id = 3
    chassis.type = 1
    # Add Marker 
    chassis.action = 0 
    
    # Set pos Marker
    chassis.pose.position.x = 9.0
    chassis.pose.position.y = 1.0
    chassis.pose.position.z = 0.4
    chassis.pose.orientation.x = 0.0
    chassis.pose.orientation.y = 0.0
    chassis.pose.orientation.z = 0.0
    chassis.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    chassis.scale.x = 0.5
    chassis.scale.y = 0.5
    chassis.scale.z = 0.5 

    # Set the color
    chassis.color.r = 1.0
    chassis.color.g = 1.0
    chassis.color.b = 0.0
    chassis.color.a = 1.0

    # Set Duration
    chassis.lifetime = rospy.Duration (0)


    # Stop Condition 
def stop() : 
        print ("Stopping")

if __name__ == "__main__":
    rospy.init_node("diferential_robot")
    loop_rate = rospy.Rate(10) #10 hz
    rospy.on_shutdown(stop)
    print("The caster_wheel is ready")
    print("The right_wheel is ready")
    print("The left_wheel is ready")
    print("The chassis is ready")

    # Setup the message 
    init_caster_wheel()
    init_wheel_r()
    init_wheel_l()
    init_chassis()
    init_world () 

    # Set publishers and subscribers 
    pub_caster = rospy.Publisher('/caster_wheel' , Marker , queue_size = 1 )
    pub_rw = rospy.Publisher('/right_wheel' , Marker , queue_size = 1 )
    pub_lw = rospy.Publisher('/left_wheel', Marker, queue_size = 1)
    pub_chassis = rospy.Publisher ('/chassis', Marker, queue_size = 1)
    pub_world = rospy.Publisher ('/world', Marker, queue_size = 1)

    try:
        while not rospy.is_shutdown() :
            caster_wheel.header.stamp = rospy.Time.now()
            wheel_l.header.stamp = rospy.Time.now()
            wheel_r.header.stamp = rospy.Time.now()
            chassis.header.stamp = rospy.Time.now()
            world.header.stamp = rospy.Time.now()
           
            pub_caster.publish(caster_wheel)
            pub_rw.publish(wheel_r)
            pub_lw.publish(wheel_l)
            pub_chassis.publish(chassis)
            pub_world.publish(world)


            loop_rate.sleep()
    except rospy.ROSInterruptException : 
        pass 