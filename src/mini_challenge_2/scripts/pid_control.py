#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32

class DiscretePIDController:
    def __init__(self):
        # Initialize node
        rospy.init_node("Discrete_PID_Controller")
        
        # PID parameters (adjustable)
        self.Kp = rospy.get_param("~Kp", 1.0)
        self.Ki = rospy.get_param("~Ki", 0.1)
        self.Kd = rospy.get_param("~Kd", 0.01)
        
        # Sampling period Ts
        self.pid_rate = rospy.get_param("~pid_rate", 100)
        self.Ts = 1.0 / self.pid_rate  # in seconds
        
        # State variables for discrete PID
        self.set_point = 0.0
        self.motor_output = 0.0
        self.error_sum = 0.0          # Error sum for integral term
        self.previous_error = 0.0     # Previous error for derivative term
        self.error_samples = []       # Store all errors for summation
        self.k = 0                    # Discrete iteration counter
        
        # Subscribers
        rospy.Subscriber("/motor_output", Float32, self.motor_output_callback)
        rospy.Subscriber("/set_point", Float32, self.set_point_callback)
        
        # Publishers
        self.motor_input_pub = rospy.Publisher("/motor_input", Float32, queue_size=10)
        
        # Update rate
        self.rate = rospy.Rate(self.pid_rate)
        
        rospy.loginfo("Discrete PID Controller initialized with:")
        rospy.loginfo(f"Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        rospy.loginfo(f"Sampling rate: {self.pid_rate}Hz (Ts={self.Ts:.4f}s)")
        rospy.loginfo("Controller will subscribe to /set_point for reference signal")

    def motor_output_callback(self, msg):
        self.motor_output = msg.data
        
    def set_point_callback(self, msg):
        self.set_point = msg.data

    def compute_discrete_pid(self):
        # Calculate current error e(k)
        current_error = self.set_point - self.motor_output
        
        # Store error history for summation
        self.error_samples.append(current_error)
        
        # Proportional term: Kp*e(k)
        P = self.Kp * current_error
        
        # Integral term: Ki*Ts*sum(e(n))
        error_sum = sum(self.error_samples)
        I = self.Ki * self.Ts * error_sum
        
        # Derivative term: Kd*(e(k) - e(k-1))/Ts
        D = 0.0
        if self.k > 0:  # Only calculate derivative if not first iteration
            D = self.Kd * (current_error - self.previous_error) / self.Ts
        
        # Save current error as previous for next iteration
        self.previous_error = current_error
        
        # Increment iteration counter
        self.k += 1
        
        # Calculate total control signal u(k)
        control_signal = P + I + D
        
        return control_signal
    
    def run(self):
        while not rospy.is_shutdown():
            # Calculate control signal using discrete equation
            control_signal = self.compute_discrete_pid()
            
            # Publish control signal
            self.motor_input_pub.publish(Float32(control_signal))
            
            # Wait according to configured frequency
            self.rate.sleep()

if __name__ == '__main__':
    try:
        pid = DiscretePIDController()
        pid.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("PID controller shutdown")