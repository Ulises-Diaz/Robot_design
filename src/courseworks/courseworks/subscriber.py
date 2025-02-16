#usr/bin/env python3 
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64

class Number_subscriber (Node): 
    def __init__ (self): 
        super().__init__('number_subscriber')
        
        # Subscriber
        self.subscription = self.create_subscription(Float64, 'number', self.listener_callback,10)
        
    def listener_callback(self,  msg):
        
        # Esto se ejecuta cada vez que reciba un mensjae
        self.get_logger().info(f'recibiendo {msg.data}')
    
def main (args= None):
    rclpy.init(args=args)
    subscriber = Number_subscriber()
    try: 
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally: 
        Number_subscriber.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
            