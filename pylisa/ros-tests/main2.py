import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('node_03')
        self.publisher_ = self.create_publisher(String, 'topic_01', 10)
        self.publisher2_ = self.create_publisher(String, 'topic_02', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('node_02')
        self.subscription = self.create_subscription(
            String,
            'topic_01',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class MinimalSubscriber2(Node):
    def __init__(self):
        super().__init__('node_01')
        self.subscription = self.create_subscription(
            String,
            'topic_02',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher2_ = self.create_publisher(Int, 'topic_02', 10)
        self.publisher2_ = self.create_publisher(String, 'topic_01', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()
