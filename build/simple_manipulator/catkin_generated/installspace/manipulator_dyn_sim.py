#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


#Declare Variables/Parameters to be used

# Setup Variables to be used

first = True 
current_time = 0.0
last_time = 0.0

tau1 = 0.0
tau2 = 0.0

robotJoints = JointState()


def input_callback(msg):
    global tau1, tau2
    tau_input = np.array([msg.data[0], msg.data[1]])
    tau1, tau2 = tau_input[0], tau_input[1]

#Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

def init_joints (): 
        robotJoints.header.frame_id = "base_link"
        robotJoints.header.stamp = rospy.Time.now()
        robotJoints.name = ["joint2", "joint3"]
        robotJoints.position = [0.0, 0.0]
        robotJoints.velocity = [0.0, 0.0]
        robotJoints.effort = [0,0]    


#Main Function
if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("DLM_Sim")

    init_joints()

    #Get DLM Parameters
    m1 = rospy.get_param("~rod1_mass",3.0)
    m2 = rospy.get_param("~rod2_mass",3.0)
    M1 = rospy.get_param("~motor_mass",1.5)
    M2 = rospy.get_param("~load_mass",1.5)

    a = rospy.get_param("~rod1_COM_pos",0.2)
    d= rospy.get_param("~rod2_COM_pos",0.2)
    l1 = rospy.get_param("~rod1_length",0.4) 
    l2 = rospy.get_param("~rod2_lenght",0.4) 


    q1 = rospy.get_param("~q1_init_angle",0.10)
    q2 = rospy.get_param("~q2_init_angle",0.1)
    q1_dot= rospy.get_param("~q1_dot_0",0.0)
    q2_dot= rospy.get_param("~q2_dot_0",0.0)

    tau1= rospy.get_param("~tau1",0.0)
    tau2= rospy.get_param("~tau2",0.0)

    gravity = rospy.get_param("~gravity",9.8)   

    # subscriber
    rospy.Subscriber('/tau', Float32MultiArray, input_callback)

    # Publisher
    state_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    print("The SLM sim is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True : 
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False 

                # system parameters
            else : 
                current_time = rospy.get_time()
                dt =  current_time - last_time
                
                    #Publish initial conditions to the joints (joint2)

                c1 = np.cos(q1)
                s1 = np.sin(q1)
                c2 = np.cos(q2)
                s2 = np.sin(q2)
                c12 = np.cos(q1+q2)
                s12 = np.sin(q1+q2)

                # MATRIXES 
                M = np.array([
                            [m1*(a**2)+M1*(l1**2) + m2*((l1**2)+(d**2) + 2*(l1*d*c2)) + M2*((l1**2) + (l2**2)+ (2*l1*l2*c2)),
                             m2*d*(l1*c2+d)+M2*l2*(l1*c2+l2)],
                            [m2*d*(l1*c2+d)+M2*l2*(l1*c2+l2),
                             m2*(d**2) + M2*(l2**2)]
                            ])
                
                C = np.array([
                            [-2*l1*s2*(m2*d + M2*l2)*q2_dot, -l1*s2*(m2*d + M2*l2)*q2_dot],
                            [l1*s2*(m2*d + M2*l2)*q1_dot, 0]
                            ])
                
                G = gravity * np.array([
                                        m1*a*c1 + M1*l1*c1 + m2*l1*c1 + m2*d*c12 + M2*l1*c1 + M2*l2*c12,
                                        (m2*d + M2*l2) * c12
                                        ])
                
                tau_vector = np.array([tau1, tau2])
                q_dot = np.array([q1_dot,q2_dot])

                M_inv = np.linalg.inv(M)

                # acceleracion 
                q_2dot_general = M_inv.dot((tau_vector - C.dot(q_dot) - G))

                q1_dot_act = q_2dot_general[0]
                q2_dot_act = q_2dot_general[1]

                # Act vel
                q1_dot = q1_dot + q1_dot_act *dt
                q2_dot = q2_dot + q2_dot_act *dt

                # Act pos
                q1 = q1 + q1_dot *dt
                q2 = q2 + q2_dot *dt

                q1 = wrap_to_Pi(q1)
                q2 = wrap_to_Pi(q2)

                #Configure joints
                robotJoints.header.stamp = rospy.Time.now()
                robotJoints.position = [q1, q2]
                robotJoints.velocity = [q1_dot, q2_dot]
                robotJoints.effort = [tau1, tau2]
                state_pub.publish(robotJoints)

                last_time = current_time

	    ## WRITE YOUR CODE HERE        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
