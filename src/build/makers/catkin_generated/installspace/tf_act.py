#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

if __name__ == '__main__':
    rospy.init_node('tf_test')
    loop_rate = rospy.Rate(10)  # rate 10 hz
    bc_tf1 = StaticTransformBroadcaster()
    bc_tf2 = TransformBroadcaster()

    # Configuración del transform del sol
    sun_tf = TransformStamped()
    sun_tf.header.stamp = rospy.Time.now()
    sun_tf.header.frame_id = 'sun'
    sun_tf.child_frame_id = 'planet'
    
    sun_tf.transform.translation.x = 5.0
    sun_tf.transform.translation.y = 3.0
    sun_tf.transform.translation.z = 6.0
    
    q = tf_conversions.transformations.quaternion_from_euler(1.57, 0.0, 3.1416)
    sun_tf.transform.rotation.x = q[0]
    sun_tf.transform.rotation.y = q[1]  # Corregido de .x a .y
    sun_tf.transform.rotation.z = q[2]  # Corregido de .x a .z
    sun_tf.transform.rotation.w = q[3]  # Corregido de .x a .w

    # Configuración inicial del transform del planeta
    planet_tf = TransformStamped()
    planet_tf.header.stamp = rospy.Time.now()
    planet_tf.header.frame_id = 'planet'
    planet_tf.child_frame_id = 'planet2'
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        
        # Actualización del transform del planeta
        planet_tf.header.stamp = rospy.Time.now()
        planet_tf.header.frame_id = 'planet'  # Corregido
        planet_tf.child_frame_id = 'planet2'
        
        planet_tf.transform.translation.x = 15.0 * np.sin(t)
        planet_tf.transform.translation.y = 32.0 * np.sin(t)
        planet_tf.transform.translation.z = 61.0
        
        q2 = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        planet_tf.transform.rotation.x = q2[0]
        planet_tf.transform.rotation.y = q2[1]  # Corregido de .x a .y
        planet_tf.transform.rotation.z = q2[2]  # Corregido de .x a .z
        planet_tf.transform.rotation.w = q2[3]  # Corregido de .x a .w
        
        bc_tf1.sendTransform(sun_tf)
        bc_tf2.sendTransform(planet_tf)
        loop_rate.sleep()