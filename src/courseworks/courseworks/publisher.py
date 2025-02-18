#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Float32
import numpy as np 

class DataPublisher(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher = self.create_publisher(Float32, '/signal', 10)
        self.time_publisher = self.create_publisher(Float32, '/time', 10)
                
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 hz
        
        # Iniciando el tiempo
        self.t = 0.0 
        
        # Generacion de se;al
        self.frequency = 1.0 # Frecuencia del sin
        self.amplitud = 1.0 # Amplitud
        
        self.get_logger().info("Sine wave Publisher Started")
        
    def timer_callback(self):
        signal_msg = Float32()
        time_msg = Float32()
        
        # Calcular el sin usando numpy
        
        signal_msg.data = float(self.amplitud * np.sin(2* np.pi *self.frequency * self.t))
        time_msg.data = self.t
        
        # Publish messages
        self.signal_publisher.publish(signal_msg)
        self.time_publisher.publish(time_msg)
        
        # Log Values
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