#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry
import tf_conversions

# Setup Variables to be used
velocity_r = 0.0
velocity_l = 0.0
omega_r = 0.0 
omega_l = 0.0 
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0

# Declare Messages to be used
pose_tf = TransformStamped()

#Initialise messages (if required)

#Initialise TF's
def init_poseTF(pos_x, pos_y, pos_th):
    #Transform the angle into a quaternion for the orientation
    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
    pose_tf.header.frame_id = "odom"
    pose_tf.child_frame_id = "base_link"
    pose_tf.header.stamp = rospy.Time.now()
    pose_tf.transform.translation.x = pos_x
    pose_tf.transform.translation.y = pos_y
    pose_tf.transform.translation.z = 0.0
    pose_tf.transform.rotation.x = pos_orient[0]
    pose_tf.transform.rotation.y = pos_orient[1]
    pose_tf.transform.rotation.z = pos_orient[2]
    pose_tf.transform.rotation.w = pos_orient[3]

#Define the callback functions (if required)
def cmd_vel_callback(msg): 
    global velocity_r, velocity_l
    
    # Convertir twist(vel) a msg llanta 
    linear_x = msg.linear.x  
    angular_z = msg.angular.z  

    # Calc vel

    velocity_r = linear_x + (angular_z * wheel_base / 2.0)
    velocity_l = linear_x - (angular_z * wheel_base / 2.0)

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("B1_sim")
 
    #Set the parameters of the system
    sample_time = rospy.get_param('~motor_sample_time', 0.01)

    # CAR PARAM. Given
    wheel_radius = rospy.get_param('~radius', 0.062)
    wheel_base = rospy.get_param('~base', 0.33)
    
    #Set initial conditions of the system (Initial position of the robot )
    pos_x = 0.00
    pos_y = 0.00
    pos_th = 0.00

    # Configure the Node
    loop_rate = rospy.Rate(200)
    rospy.on_shutdown(stop)

    #Init messages to be used
    init_poseTF(pos_x, pos_y, pos_th)

    ''' MIO '''

    # Setup the Subscribers
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    
    #Setup de publishers
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 10)

    '''                        '''
    #Setup TF Broadcasters
    pose_tf_bc = TransformBroadcaster()

    #Node Running
    print("The Robot Simulator is Running")

    try:
    # Run the node
        while not rospy.is_shutdown():
            

            # para el time sampling

            if first:
                start_time = rospy.get_time()
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False
            else:
                current_time = rospy.get_time()
                dt = current_time - last_time

            # Eq Diff Calc robot con euler
                if dt >= sample_time:
                    pos_x = pos_x + dt * (((velocity_r + velocity_l) / 2) * np.cos(pos_th))
                    pos_y = pos_y + dt * (((velocity_r + velocity_l) / 2) * np.sin(pos_th))
                    pos_th = pos_th + dt * ((velocity_r - velocity_l) / wheel_base)

                # Act tiempo para siguiente iteracion
                    last_time = current_time

                # Pasar angulo a Quat para ros
                    q_steering = tf_conversions.transformations.quaternion_from_euler(0, 0, pos_th)

                # Tf msg carro
                    pose_tf.header.stamp = rospy.Time.now()
                    pose_tf.transform.translation.x = pos_x
                    pose_tf.transform.translation.y = pos_y
                    pose_tf.transform.rotation.x = q_steering[0]
                    pose_tf.transform.rotation.y = q_steering[1]
                    pose_tf.transform.rotation.z = q_steering[2]
                    pose_tf.transform.rotation.w = q_steering[3]

                # Publicar tfs
                    pose_tf_bc.sendTransform(pose_tf)

                #  Msg odometria 

                # ODOM : POSE, TWIST
                    odom_msg = Odometry()
                    odom_msg.header.stamp = rospy.Time.now()
                    odom_msg.header.frame_id = "odom"
                    odom_msg.child_frame_id = "base_link"

                # POS : Position,  Orientation
                    odom_msg.pose.pose.position.x = pos_x
                    odom_msg.pose.pose.position.y = pos_y
                    odom_msg.pose.pose.position.z = 0.0

                # Orientación
                    odom_msg.pose.pose.orientation.x = q_steering[0]
                    odom_msg.pose.pose.orientation.y = q_steering[1]
                    odom_msg.pose.pose.orientation.z = q_steering[2]
                    odom_msg.pose.pose.orientation.w = q_steering[3]

                # TWIST velocidades: Angular, linear
                    odom_msg.twist.twist.linear.x = (velocity_r + velocity_l) / 2.0
                    odom_msg.twist.twist.angular.z = (velocity_r - velocity_l) / wheel_base

                # Publicar odometría
                    odom_pub.publish(odom_msg)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass
