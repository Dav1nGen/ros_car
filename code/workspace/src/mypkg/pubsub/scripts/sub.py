#!/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber_py')
        self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('接收到: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

