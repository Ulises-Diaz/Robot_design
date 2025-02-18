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
        self.phase_shift = 0.0  # Fase en radianes, ajustable por el usuario

        # Suscriptores
        self.subscription_signal = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.subscription_time = self.create_subscription(Float32, '/time', self.time_callback, 10)
        
        
        # Publicador
        self.publisher = self.create_publisher(Float32, '/proc_signal', 10)
        
        # Nuevo subscriber para /proc_signal
        self.subscription_proc = self.create_subscription(Float32, '/proc_signal', self.proc_signal_callback, 10)


        # Timer para procesar la señal a 10 Hz
        self.timer = self.create_timer(0.1, self.processed_published_signal)

    def signal_callback(self, msg):
        """Callback para el topic '/signal'."""
        self.last_signal = msg.data

    def time_callback(self, msg):
        """Callback para el topic '/time'."""
        self.last_time = msg.data

    def proc_signal_callback(self, msg):
        """Callback para el topic '/proc_signal'."""
        self.get_logger().info(f'Señal procesada recibida: {msg.data}')

    def processed_published_signal(self):
        
        #Procesa la señal y la publica en '/proc_signal'
        
        
        if self.last_signal is not None and self.last_time is not None:
            # Aplicar el desfase de fase a la señal
            shifted_signal = math.sin(self.last_time + self.phase_shift)
            # Reducir la amplitud a la mitad
            reduced_amplitude_signal = 0.5 * shifted_signal
            # Aplicar el offset para que sea siempre positiva
            processed_signal = reduced_amplitude_signal + 0.5
            # Publicar la señal procesada
            msg = Float32()
            msg.data = processed_signal
            self.publisher.publish(msg)
            # Imprimir la señal resultante en la terminal
            self.get_logger().info(f'Señal procesada publicada: {processed_signal}')

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