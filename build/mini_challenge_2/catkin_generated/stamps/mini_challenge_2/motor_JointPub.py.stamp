#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class MotorJointPublisher:
    def __init__(self):
        # Initialize node
        rospy.init_node("DC_MotorJoints")
        
        # Parameters
        self.sample_time = rospy.get_param("~sample_time", 0.01)
        self.joint_name = "joint2"  # Must match URDF joint name
        
        # State variables
        self.motor_angle = 0.0
        self.motor_output = 0.0
        self.last_time = rospy.get_time()
        
        # JointState message setup
        self.joint_msg = JointState()
        self.joint_msg.name = [self.joint_name]
        self.joint_msg.position = [0.0]
        self.joint_msg.velocity = [0.0]  # Critical addition!
        self.joint_msg.effort = [0.0]
        
        # Setup ROS interfaces
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/motor_output", Float32, self.motor_output_callback)
        
        rospy.loginfo("Motor Joint Publisher initialized for joint: %s", self.joint_name)

    def motor_output_callback(self, msg):
        self.motor_output = msg.data

    def wrap_to_pi(self, theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

    def run(self):
        rate = rospy.Rate(rospy.get_param("~publish_rate", 100))
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            dt = current_time - self.last_time
            
            if dt >= self.sample_time:
                # Update position (integrate velocity)
                self.motor_angle += self.motor_output * dt
                
                # Prepare joint state message
                self.joint_msg.header.stamp = rospy.Time.now()
                self.joint_msg.position[0] = self.wrap_to_pi(self.motor_angle)
                self.joint_msg.velocity[0] = self.motor_output  # Critical!
                
                # Publish
                self.joint_pub.publish(self.joint_msg)
                
                self.last_time = current_time
            
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = MotorJointPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Motor joint publisher shutdown")