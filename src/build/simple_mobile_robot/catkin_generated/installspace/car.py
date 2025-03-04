#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions
from visualization_msgs.msg import Marker

# Setup parameters, variables and callback functions
# Declare msg

'''

Declare Markers 

'''

caster_wheel_marker = Marker()
wheel_r_marker = Marker()
wheel_l_marker = Marker()
chassis_marker = Marker()
world_marker = Marker()
base_link_marker = Marker()

'''

Declare Transforms

'''

world_tf = TransformStamped()
base_link_tf = TransformStamped()
wheel_l_tf = TransformStamped()
wheel_r_tf = TransformStamped()
caster_wheel_tf = TransformStamped()

'''

INIT Markers 

'''

def init_world(): 
    # Header
    world_marker.header.frame_id = "world" # nombre del fixed frame
    world_marker.header.stamp = rospy.Time.now() 

    # Set shape 
    world_marker.id = 0
    world_marker.type = 2
    # Add Marker 
    world_marker.action = 0 
    
    # Set pos Marker
    world_marker.pose.position.x = 0.0
    world_marker.pose.position.y = 0.0
    world_marker.pose.position.z = 0.0
    world_marker.pose.orientation.x = 0.0
    world_marker.pose.orientation.y = 0.0
    world_marker.pose.orientation.z = 0.0
    world_marker.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    world_marker.scale.x = 0.5
    world_marker.scale.y = 0.5
    world_marker.scale.z = 0.5 

    # Set the color
    world_marker.color.r = 1.0
    world_marker.color.g = 1.0
    world_marker.color.b = 0.0
    world_marker.color.a = 1.0

    # Set Duration
    world_marker.lifetime = rospy.Duration(0)

def init_caster_wheel(): 
    # Header
    caster_wheel_marker.header.frame_id = "caster_wheel" # nombre del fixed frame
    caster_wheel_marker.header.stamp = rospy.Time.now() 

    # Set shape 
    caster_wheel_marker.id = 5
    caster_wheel_marker.type = 2
    # Add Marker 
    caster_wheel_marker.action = 0 
    
    # Set pos Marker
    caster_wheel_marker.pose.position.x = 0.0
    caster_wheel_marker.pose.position.y = 0.0
    caster_wheel_marker.pose.position.z = 0.0
    caster_wheel_marker.pose.orientation.x = 0.0
    caster_wheel_marker.pose.orientation.y = 0.0
    caster_wheel_marker.pose.orientation.z = 0.0
    caster_wheel_marker.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    caster_wheel_marker.scale.x = 0.15
    caster_wheel_marker.scale.y = 0.15
    caster_wheel_marker.scale.z = 0.15

    # Set the color
    caster_wheel_marker.color.r = 1.0
    caster_wheel_marker.color.g = 1.0
    caster_wheel_marker.color.b = 4.0
    caster_wheel_marker.color.a = 1.0

    # Set Duration
    caster_wheel_marker.lifetime = rospy.Duration(0)

def init_wheel_r(): 
    # Header
    wheel_r_marker.header.frame_id = "wheel_r" # nombre del fixed frame
    wheel_r_marker.header.stamp = rospy.Time.now() 

    # Set shape 
    wheel_r_marker.id = 4
    wheel_r_marker.type = 2
    # Add Marker 
    wheel_r_marker.action = 0 
    
    # Set pos Marker
    wheel_r_marker.pose.position.x = 0.0
    wheel_r_marker.pose.position.y = 0.0
    wheel_r_marker.pose.position.z = 0.0
    wheel_r_marker.pose.orientation.x = 0.0
    wheel_r_marker.pose.orientation.y = 0.0
    wheel_r_marker.pose.orientation.z = 0.0
    wheel_r_marker.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    wheel_r_marker.scale.x = 0.2
    wheel_r_marker.scale.y = 0.2
    wheel_r_marker.scale.z = 0.2 

    # Set the color
    wheel_r_marker.color.r = 2.0
    wheel_r_marker.color.g = 3.4
    wheel_r_marker.color.b = 0.0
    wheel_r_marker.color.a = 1.0

    # Set Duration
    wheel_r_marker.lifetime = rospy.Duration(0)

def init_wheel_l(): 
    # Header
    wheel_l_marker.header.frame_id = "wheel_l" # nombre del fixed frame
    wheel_l_marker.header.stamp = rospy.Time.now() 

    # Set shape 
    wheel_l_marker.id = 3
    wheel_l_marker.type = 2
    # Add Marker 
    wheel_l_marker.action = 0 
    
    # Set pos Marker
    wheel_l_marker.pose.position.x = 0.0
    wheel_l_marker.pose.position.y = 0.0
    wheel_l_marker.pose.position.z = 0.0
    wheel_l_marker.pose.orientation.x = 0.0
    wheel_l_marker.pose.orientation.y = 0.0
    wheel_l_marker.pose.orientation.z = 0.0
    wheel_l_marker.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    wheel_l_marker.scale.x = 0.2
    wheel_l_marker.scale.y = 0.2
    wheel_l_marker.scale.z = 0.2 

    # Set the color
    wheel_l_marker.color.r = 4.0
    wheel_l_marker.color.g = 1.0
    wheel_l_marker.color.b = 0.0
    wheel_l_marker.color.a = 1.0

    # Set Duration
    wheel_l_marker.lifetime = rospy.Duration(0)

def init_chassis(): 
    # Header
    chassis_marker.header.frame_id = "chassis" # nombre del fixed frame
    chassis_marker.header.stamp = rospy.Time.now() 

    # Set shape 
    chassis_marker.id = 2
    chassis_marker.type = 1
    # Add Marker 
    chassis_marker.action = 0 
    
    # Set pos Marker
    chassis_marker.pose.position.x = 0.0
    chassis_marker.pose.position.y = 0.0
    chassis_marker.pose.position.z = 0.0
    chassis_marker.pose.orientation.x = 0.0
    chassis_marker.pose.orientation.y = 0.0
    chassis_marker.pose.orientation.z = 0.0
    chassis_marker.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    chassis_marker.scale.x = 0.4
    chassis_marker.scale.y = 0.4
    chassis_marker.scale.z = 0.4 

    # Set the color
    chassis_marker.color.r = 0.0
    chassis_marker.color.g = 3.0
    chassis_marker.color.b = 1.0
    chassis_marker.color.a = 1.0

    # Set Duration
    chassis_marker.lifetime = rospy.Duration(0) 

def init_base_link():

    base_link_marker.header.frame_id = "base_link" # nombre del fixed frame
    base_link_marker.header.stamp = rospy.Time.now() 

    # Set shape 
    base_link_marker.id =  1
    base_link_marker.type = 1
    # Add Marker 
    base_link_marker.action = 0 
    
    # Set pos Marker
    base_link_marker.pose.position.x = 0.0
    base_link_marker.pose.position.y = 0.0
    base_link_marker.pose.position.z = 0.0
    base_link_marker.pose.orientation.x = 0.0
    base_link_marker.pose.orientation.y = 0.0
    base_link_marker.pose.orientation.z = 0.0
    base_link_marker.pose.orientation.w = 1.0

    # Set the scale of the marker 
    # Change dimensions 
    base_link_marker.scale.x = 0.2
    base_link_marker.scale.y = 0.2
    base_link_marker.scale.z = 0.2 

    # Set the color
    base_link_marker.color.r = 1.0
    base_link_marker.color.g = 0.0
    base_link_marker.color.b = 1.0
    base_link_marker.color.a = 1.0

    # Set Duration
    base_link_marker.lifetime = rospy.Duration(0) 

    # Stop Condition 

'''

Init Transforms

'''

def init_world_tf():
    world_tf.header.frame_id = 'world'
    world_tf.child_frame_id = 'base_link' 
    world_tf.header.stamp = rospy.Time.now()
    world_tf.transform.translation.x = 0.0
    world_tf.transform.translation.y = 0.0
    world_tf.transform.translation.z = 0.0
    world_tf.transform.rotation.x = 0.0
    world_tf.transform.rotation.y = 0.0
    world_tf.transform.rotation.z = 0.0
    world_tf.transform.rotation.w = 1.0

def init_base_link_tf():

    base_link_tf.header.frame_id = 'base_link'
    base_link_tf.child_frame_id = 'chassis'
    base_link_tf.header.stamp = rospy.Time.now()
    base_link_tf.transform.translation.x = 0.0
    base_link_tf.transform.translation.y = 0.0
    base_link_tf.transform.translation.z = 0.2
    base_link_tf.transform.rotation.x = 0.0
    base_link_tf.transform.rotation.y = 0.0
    base_link_tf.transform.rotation.z = 0.0
    base_link_tf.transform.rotation.w = 1.0

def init_wheel_l_tf():

    wheel_l_tf.header.frame_id = 'chassis'
    wheel_l_tf.child_frame_id = 'wheel_l'
    wheel_l_tf.header.stamp = rospy.Time.now()
    wheel_l_tf.transform.translation.x = 0.1
    wheel_l_tf.transform.translation.y = 0.2
    wheel_l_tf.transform.translation.z = -0.2
    wheel_l_tf.transform.rotation.x = 0.0
    wheel_l_tf.transform.rotation.y = 0.0
    wheel_l_tf.transform.rotation.z = 0.0
    wheel_l_tf.transform.rotation.w = 1.0

def init_wheel_r_tf():

    wheel_r_tf.header.frame_id = 'chassis'
    wheel_r_tf.child_frame_id = 'wheel_r'
    wheel_r_tf.header.stamp = rospy.Time.now()
    wheel_r_tf.transform.translation.x = 0.1
    wheel_r_tf.transform.translation.y = -0.2
    wheel_r_tf.transform.translation.z = -0.2
    wheel_r_tf.transform.rotation.x = 0.0
    wheel_r_tf.transform.rotation.y = 0.0
    wheel_r_tf.transform.rotation.z = 0.0
    wheel_r_tf.transform.rotation.w = 1.0

def init_caster_wheel_tf():

    caster_wheel_tf.header.frame_id = 'chassis'
    caster_wheel_tf.child_frame_id = 'caster_wheel'
    caster_wheel_tf.header.stamp = rospy.Time.now()
    caster_wheel_tf.transform.translation.x = -0.1
    caster_wheel_tf.transform.translation.y = 0.0
    caster_wheel_tf.transform.translation.z = -0.225
    caster_wheel_tf.transform.rotation.x = 0.0
    caster_wheel_tf.transform.rotation.y = 0.0
    caster_wheel_tf.transform.rotation.z = 0.0
    caster_wheel_tf.transform.rotation.w = 1.0


def stop(): 
        print("Stopping")

if __name__ == "__main__":
    rospy.init_node("differential_robot")
    loop_rate = rospy.Rate(10) #10 hz
    rospy.on_shutdown(stop)
    print("The caster_wheel is ready")
    print("The right_wheel is ready")
    print("The left_wheel is ready")
    print("The chassis is ready")
    print("World and base_link are ready")

    # Setup Markers
    init_base_link()
    init_caster_wheel()
    init_wheel_r()
    init_wheel_l()
    init_chassis()
    init_world() 

    # Setup transformers
    init_world_tf()
    init_base_link_tf() 
    init_wheel_l_tf()
    init_wheel_r_tf()
    init_caster_wheel_tf()

    # Set movement of the car
    speed = 1
    radius = 2
    length_axis = 0.6

    # Set publishers and subscribers Markers
    pub_caster = rospy.Publisher('/caster_wheel', Marker, queue_size=1)
    pub_rw = rospy.Publisher('/right_wheel', Marker, queue_size=1)
    pub_lw = rospy.Publisher('/left_wheel', Marker, queue_size=1)
    pub_chassis = rospy.Publisher('/chassis', Marker, queue_size=1)
    pub_world = rospy.Publisher('/world', Marker, queue_size=1)
    pub_base_link = rospy.Publisher('/base_link', Marker, queue_size=1)

    # Set publisher and subscribers Transforms
    bc_world = TransformBroadcaster()
    bc_base_link = StaticTransformBroadcaster()
    bc_chassis = TransformBroadcaster()
    bc_wheel_r = TransformBroadcaster()
    bc_wheel_l = TransformBroadcaster()
    bc_caster = StaticTransformBroadcaster()

    try:
        while not rospy.is_shutdown():

            t = rospy.Time.now().to_sec()

            # Markers
            caster_wheel_marker.header.stamp = rospy.Time.now()
            wheel_l_marker.header.stamp = rospy.Time.now()
            wheel_r_marker.header.stamp = rospy.Time.now()
            chassis_marker.header.stamp = rospy.Time.now()
            world_marker.header.stamp = rospy.Time.now()

            # Define movement
            x_pos = radius * np.cos(speed * t)
            y_pos = radius * np.sin(speed * t)

            vel = speed * radius
            angular_vel = speed 
            steering_angle = np.arctan(length_axis * angular_vel / vel) # Angulo de giro 

            # Rotation movement (row, pitch, yaw)
            q_world_base = tf_conversions.transformations.quaternion_from_euler(0, 0, speed * t + np.pi / 2)
            q_steering = tf_conversions.transformations.quaternion_from_euler(0, 0, steering_angle)

            # Update las transformadas
            world_tf.header.stamp = rospy.Time.now()
            world_tf.transform.translation.x = x_pos
            world_tf.transform.translation.y = y_pos
            world_tf.transform.rotation.x = q_world_base[0]
            world_tf.transform.rotation.y = q_world_base[1]
            world_tf.transform.rotation.z = q_world_base[2]
            world_tf.transform.rotation.w = q_world_base[3]

            # Aplicar mov a las llantas
            '''
            llanta derecha
            '''
            wheel_r_tf.header.stamp = rospy.Time.now()
            wheel_r_tf.transform.rotation.x = q_steering[0]
            wheel_r_tf.transform.rotation.y = q_steering[1]
            wheel_r_tf.transform.rotation.z = q_steering[2]
            wheel_r_tf.transform.rotation.w = q_steering[3]

            '''
            llanta izq
            '''
            wheel_l_tf.header.stamp = rospy.Time.now()
            wheel_l_tf.transform.rotation.x = q_steering[0]
            wheel_l_tf.transform.rotation.y = q_steering[1]
            wheel_l_tf.transform.rotation.z = q_steering[2]
            wheel_l_tf.transform.rotation.w = q_steering[3]

            # Markers publishers
            pub_caster.publish(caster_wheel_marker)
            pub_rw.publish(wheel_r_marker)
            pub_lw.publish(wheel_l_marker)
            pub_chassis.publish(chassis_marker)
            pub_base_link.publish(base_link_marker)
            pub_world.publish(world_marker)

            # Transform publishers
            bc_world.sendTransform(world_tf)
            bc_base_link.sendTransform(base_link_tf)
            bc_caster.sendTransform(caster_wheel_tf)
            bc_wheel_l.sendTransform(wheel_l_tf)
            bc_wheel_r.sendTransform(wheel_r_tf)

            loop_rate.sleep()
    except rospy.ROSInterruptException: 
        pass