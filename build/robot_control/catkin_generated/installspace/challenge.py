#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

# Global Variables
first = True
posx = 0.0
posy = 0.0
angle = 0.0
velocity = 0.5
angular_vel = 0.0
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0
robot_name = "puzzlebot"
kp = 5.0
ki = 0.0
kd = 0.1
past_error = 0.0
sum_error = 0.0

# para hacer cuadrado
targets = [(2.0, -2.0),(2.0, 2.0),(-2.0, 2.0),(-2.0, -2.0)]
target_index = 0  # Current target index

'''
#Triangulo
targets = [
    (2.0, 0.0),           
    (-1.0, 1.732),        
    (-1.0, -1.732)        
]

target_index = 0  # Current target index

'''

'''
# Rombo
targets = [
    (2.0, 0.0),           # Vértice derecho
    (0.0, 2.0),           # Vértice superior
    (-2.0, 0.0),          # Vértice izquierdo
    (0.0, -2.0)           # Vértice inferior
]

target_index = 0 
'''

# Message declaration
controlOutput = Twist()
clock_msg = None

def Model_States_callback(msg):
    global posx, posy, angle
    
    # Buscar nuestro robot en el mensaje
    if robot_name in msg.name:
        idx = msg.name.index(robot_name)
        
        # Actualizar posición
        posx = msg.pose[idx].position.x
        posy = msg.pose[idx].position.y
        
        # Extraer orientación del quaternion
        quat = msg.pose[idx].orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        
        # Convertir quaternion a ángulos de Euler (roll, pitch, yaw)
        roll, pitch, angle = euler_from_quaternion(quat_list)

def PID():
    global targets, target_index, posx, posy, angle, angular_vel, past_error, sum_error, kp, ki, kd, dt
    errorx = targets[target_index][0] - posx
    errory = targets[target_index][1] - posy
    
    # Calculate target angle 
    target_angle = np.arctan2(errory, errorx)  
    
    # Calculate angular error
    error_angle = target_angle - angle
    
    # Mantener el error entre 180 y -180
    if error_angle > np.pi:
        error_angle -= 2.0 * np.pi
    elif error_angle < -np.pi:
        error_angle += 2.0 * np.pi 

    # PID Controller
    P = kp * error_angle
    I = ki * sum_error * dt
    D = kd * (error_angle - past_error) / dt

    #velocidad angular
    angular_vel = P + I + D
    
    past_error = error_angle
    sum_error += error_angle * dt

def stop():
    controlOutput.linear.x = 0.0
    controlOutput.angular.z = 0.0
    control_pub.publish(controlOutput)
    print("stopping")

def clock_callback(msg):
    global clock_msg
    clock_msg = msg

if __name__ == '__main__':
    rospy.init_node("Controlador_puzzlebot")
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 10))
    rospy.on_shutdown(stop)

    # Publishers
    control_pub = rospy.Publisher("/puzzlebot/cmd_vel", Twist, queue_size=1)

    # Subscribers
    clock_sub = rospy.Subscriber('/clock', Clock, clock_callback, queue_size=1)
    ms_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, Model_States_callback, queue_size=1)

    print("The controller is Running")

    try:
        # Wait for Gazebo initialization
        while clock_msg is None:
            rospy.loginfo("Waiting for Gazebo")
            loop_rate.sleep()

        rospy.loginfo("Clock synchronized")
        rospy.sleep(0.5)

        past_time = rospy.Time.now().to_sec()

        # Main control loop
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            dt = current_time - last_time
            last_time = current_time

            if dt <= 0:
                dt = 0.0000000001

            # Hacer PID
            PID ()
            
            # PPublicar velocidades
            controlOutput.linear.x = velocity
            controlOutput.angular.z = angular_vel
            control_pub.publish(controlOutput)

            distancia_x = abs(targets[target_index][0] - posx)
            distancia_y = abs(targets[target_index][1] - posy)

            # Tolerancia de 10cm para considerar que llegamos
            if distancia_x < 0.1 and distancia_y < 0.1:
                rospy.loginfo("Checkpoint alcanzado")
                
                # Verificar si hay mas puntos
                if target_index < len(targets) - 1:
                    target_index += 1
                    rospy.loginfo("nuevo punto")
                else:
                    rospy.loginfo("Figura terminada")
                    stop()
                    

            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass