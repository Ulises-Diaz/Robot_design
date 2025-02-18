#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SignalSubscriber(Node):
    def __init__(self):
        super().__init__('process')
        
        # Variables para almacenar los valores más recientes
        
        self.last_signal = None
        self.last_time = None
        self.phase_shift = 0.0  # Fase en radianes, puede modificarse

        # Suscriptores
        self.subscription_signal = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.subscription_time = self.create_subscription(Float32, '/time', self.time_callback, 10)
        
        
        # Publisher
        self.publisher = self.create_publisher(Float32, '/proc_signal', 10)
        
        #  subscriber para /proc_signal
        self.subscription_procesado = self.create_subscription(Float32, '/proc_signal', self.proc_signal_callback, 10)


        # Timer para procesar la señal a 10 Hz
        self.timer = self.create_timer(0.1, self.processed_published_signal)

    def signal_callback(self, msg):
        # Callback topico /signal
        self.last_signal = msg.data

    def time_callback(self, msg):
        # Callback topico '/time
        self.last_time = msg.data

    def proc_signal_callback(self, msg):
        # Callback topico '/proc_signal
        self.get_logger().info(f'Señal procesada recibida: {msg.data}')

    def processed_published_signal(self):
        
        #Procesa la señal y la publica en '/proc_signal'
        
        if self.last_signal is not None and self.last_time is not None:
            # Aplicar el desfase de fase a la señal
            senal_desfase = math.sin(self.last_time + self.phase_shift)
            
            # Reducir la amplitud a la mitad
            senal_reducida = 0.5 * senal_desfase
            
            # Aplicar el offset para que sea siempre positiva
            senal_procesada = senal_reducida + 0.5
            
            # Publicar la señal procesada
            msg = Float32()
            msg.data = senal_procesada
            self.publisher.publish(msg)
            
            # Imprimir la señal resultante en la terminal
            self.get_logger().info(f'Señal procesada publicada: {senal_procesada}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = SignalSubscriber()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()