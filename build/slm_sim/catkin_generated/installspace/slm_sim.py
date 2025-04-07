#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


# Setup Variables to be used
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0


# Declare the input Message
torque = Float32()
torque.data = 0.0

# Declare the output message
slmJoints = JointState()

#Define the callback functions
def input_callback (msg):
    global torque
    torque = msg


#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get SLM Parameters
    sample_time = rospy.get_param("~sample_time",0.01)
    friction = rospy.get_param("~friction_coeficient",0.01)
    mass = rospy.get_param("~rod_mass",3.0)
    length = rospy.get_param("~rod_length",0.4)
    q = rospy.get_param("~intial_angle",0.0)
    q_dot= rospy.get_param("~initial_angular_vel",0.0)
    gravity = rospy.get_param("~gravity",9.8)        

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~rate", 100))
    rospy.on_shutdown(stop)

    # Setup the Subscribers
    rospy.Subscriber("/tau",Float32,input_callback)

    #Setup de publishers
    state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    print("The SLM sim is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True:
                # System parameters
                center_mass_a = length/2
                J = (4/3)*mass*np.power(center_mass_a,2)
                # Initialise Joint States
                x1 = q 
                x2 = q_dot

                slmJoints.header.frame_id = "base"
                slmJoints.header.stamp = rospy.Time.now()
                slmJoints.name = ["joint2"]
                slmJoints.position = [x1]
                slmJoints.velocity = [x2]
                slmJoints.effort = [0,0]
                state_pub.publish(slmJoints)
                    #Publish initial conditions to the joints (joint2)
                #Initialise time and variables
                first = False
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                last_time = rospy.get_time()                

        #System
            else:
            #Define sampling time
                current_time = rospy.get_time()
                dt = current_time - last_time
            #Dynamical System Simulation
                if dt >= sample_time:                   

                    x1 += dt * (x2)

                    x2_dot = (1/J)*(torque.data-mass*gravity*np.cos(x1) - friction*x2)
                    x2 += dt * x2_dot
                
                    #Message to publish
                    slmJoints.header.stamp = rospy.Time.now()
                    slmJoints.position[0] = x1
                    slmJoints.velocity[0] = x2
                    slmJoints.effort[0] = 0.0

                    #Publish message
                    state_pub.publish(slmJoints)

                    #Get the previous time
                    last_time = rospy.get_time()
        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node