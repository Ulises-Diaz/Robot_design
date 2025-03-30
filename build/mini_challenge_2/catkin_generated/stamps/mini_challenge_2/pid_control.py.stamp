#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class PIDController:
    def __init__(self):
        # Inicializar nodo
        rospy.init_node("PID_Controller")

        # Parámetros PID (ajustables)
        self.Kp = rospy.get_param("~Kp", 1.0)
        self.Ki = rospy.get_param("~Ki", 0.1)
        self.Kd = rospy.get_param("~Kd", 0.01)

        # Variables de estado
        self.set_point = 0.0
        self.motor_output = 0.0
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = rospy.get_time()

        # Suscriptores
        rospy.Subscriber("/set_point", Float32, self.set_point_callback)
        rospy.Subscriber("/motor_output", Float32, self.motor_output_callback)

        # Publicador
        self.motor_input_pub = rospy.Publisher("/motor_input", Float32, queue_size=10)

        # Frecuencia de actualización
        self.rate = rospy.Rate(rospy.get_param("~pid_rate", 100))

    def set_point_callback(self, msg):
        self.set_point = msg.data

    def motor_output_callback(self, msg):
        self.motor_output = msg.data

    def compute_pid(self):
        current_time = rospy.get_time()
        dt = current_time - self.last_time

        # Cálculo de error
        error = self.set_point - self.motor_output

        # Término proporcional
        P = self.Kp * error

        # Término integral
        self.integral_error += error * dt
        I = self.Ki * self.integral_error

        # Término derivativo
        error_derivative = (error - self.last_error) / dt if dt > 0 else 0
        D = self.Kd * error_derivative

        # Guardar estado para próxima iteración
        self.last_error = error
        self.last_time = current_time

        return P + I + D

    def run(self):
        while not rospy.is_shutdown():
            # Calcular señal de control
            control_signal = self.compute_pid()

            # Publicar señal de control
            self.motor_input_pub.publish(Float32(control_signal))

            self.rate.sleep()

if __name__ == '__main__':
    pid = PIDController()
    pid.run()