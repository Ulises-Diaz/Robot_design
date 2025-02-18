#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math  # Usar la librería math en lugar de numpy

class DataPublisher(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher = self.create_publisher(Float32, '/signal', 10)
        self.time_publisher = self.create_publisher(Float32, '/time', 10)
                
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Iniciando el tiempo
        self.t = 0.0 
        
        # Generación de señal
        self.frequency = 1.0  # Frecuencia del seno
        self.amplitud = 1.0  # Amplitud
        
        self.get_logger().info("Publisher of the Sine wave Started")
        
    def timer_callback(self):
        # Asignar como Float los valores de los mensajes 
        signal_msg = Float32()
        time_msg = Float32()
        
        # Usar math.sin para calcular el seno
        signal_msg.data = float(self.amplitud * math.sin(2 * math.pi * self.frequency * self.t))
        time_msg.data = self.t
        
        # Publicar mensajes 
        self.signal_publisher.publish(signal_msg)
        self.time_publisher.publish(time_msg)
        
        # Mostrar los valores en el log
        self.get_logger().info(f" Time : {self.t:.2f}, Signal : {signal_msg.data:.2f}")
        
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
