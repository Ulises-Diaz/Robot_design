#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(Float64, 'number', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        msg = Float64()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1.0

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