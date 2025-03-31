#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32

# Variables globales
Kp = 1.0
Ki = 0.1
Kd = 0.01
pid_rate = 100
Ts = 1.0 / pid_rate
set_point = 0.0
motor_output = 0.0
error_sum = 0.0
previous_error = 0.0
error_samples = []
k = 0
motor_input_pub = None

def motor_output_callback(msg):
    global motor_output
    motor_output = msg.data
        
def set_point_callback(msg):
    global set_point
    set_point = msg.data

def wrap_to_pi(theta):
    result = math.fmod((theta + math.pi), (2 * math.pi))
    if result < 0:
        result += 2 * math.pi
    return result - math.pi

def compute_discrete_pid():
    global error_sum, previous_error, error_samples, k
    
    # Calculate current error e(k)
    current_error = set_point - motor_output
    
    # Store error history for summation
    error_samples.append(current_error)
    
    # Proportional term: Kp*e(k)
    P = Kp * current_error
    
    # Integral term: Ki*Ts*sum(e(n))
    current_error_sum = sum(error_samples)
    I = Ki * Ts * current_error_sum
    
    # Derivative term: Kd*(e(k) - e(k-1))/Ts
    D = 0.0
    if k > 0:  # Only calculate derivative if not first iteration
        D = Kd * (current_error - previous_error) / Ts
    
    # Save current error as previous for next iteration
    previous_error = current_error
    
    # Increment iteration counter
    k += 1
    
    # Calculate total control signal u(k)
    control_signal = P + I + D
    
    return control_signal

def init_pid():
    global Kp, Ki, Kd, pid_rate, Ts, motor_input_pub
    
    # Initialize node
    rospy.init_node("Discrete_PID_Controller")
    
    # Get parameters
    Kp = rospy.get_param("~Kp", 1.0)
    Ki = rospy.get_param("~Ki", 0.1)
    Kd = rospy.get_param("~Kd", 0.01)
    pid_rate = rospy.get_param("~pid_rate", 100)
    Ts = 1.0 / pid_rate
    
    # Setup publishers and subscribers
    motor_input_pub = rospy.Publisher("/motor_input", Float32, queue_size=10)
    rospy.Subscriber("/motor_output", Float32, motor_output_callback)
    rospy.Subscriber("/set_point", Float32, set_point_callback)
    
    rospy.loginfo("Discrete PID Controller initialized with:")
    rospy.loginfo(f"Kp={Kp}, Ki={Ki}, Kd={Kd}")
    rospy.loginfo(f"Sampling rate: {pid_rate}Hz (Ts={Ts:.4f}s)")

def run_pid():
    rate = rospy.Rate(pid_rate)
    while not rospy.is_shutdown():
        # Calculate control signal
        control_signal = compute_discrete_pid()
        
        # Publish control signal
        motor_input_pub.publish(Float32(control_signal))
        
        # Wait according to configured frequency
        rate.sleep()

if __name__ == '__main__':
    try:
        init_pid()
        run_pid()
    except rospy.ROSInterruptException:
        rospy.loginfo("PID controller shutdown")