#!/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher_py')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # seconds
        timer_period = 1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '这里是第%d条信息' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('发布: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

