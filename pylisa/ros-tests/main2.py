import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        x = "node_01"
        super().__init__(x)
        self.subscription = self.create_subscription(
            String,
            'topic_01',
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'topic_02',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Int, 'topic_03', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class MinimalSubscriber2(Node):
    def __init__(self):
        self.x = 3
        y = "test_constant_prop"
        super().__init__(y)
        self.subscription = self.create_subscription(
            String,
            'topic_02',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Int, 'topic_01', 10)
        self.publisher = self.create_publisher(String, 'topic_02', 10)
        self.publisher = self.create_publisher(Boolean, 'topic_03', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node1 = MinimalSubscriber()
    node2 = MinimalSubscriber2()

if __name__ == '__main__':
    main()
