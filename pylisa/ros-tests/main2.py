import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROSNode1(Node):

    def __init__(self):
        self.x = 3
        super().__init__('node_1')
        self.subscription = self.create_subscription(
            String,
            'topic1',
            self.listener_callback,
            10)
        self.create_publisher(
            String,
            'topic7',
            10)
        self.create_publisher(
            String,
            'topic1',
            10)
        self.create_subscription(
            String,
            'topic3',
            self.listener_callback,
            10)
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class ROSNode2(Node):

    def __init__(self):
        super().__init__('node_2')
        self.publisher = self.create_publisher(String, 'topic1', 10)
        self.create_publisher(String, 'topic2', 10)
        self.create_publisher(String, 'topic3', 10)
        self.create_publisher(String, 'topic4', 10)
        self.create_publisher(String, 'topic5', 10)
        self.create_publisher(String, 'topic6', 10)
        self.create_publisher(String, 'topic7', 10)
        self.create_subscription(String, 'topic7', self.topic7_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def topic7_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class ROSNode3(Node):

    def __init__(self):
        super().__init__('node_3')
        self.create_subscription(String, 'topic2', self.topic2_callback, 10)
        self.create_subscription(String, 'topic3', self.topic3_callback, 10)
        self.create_subscription(String, 'topic6', self.topic6_callback, 10)
        self.create_subscription(String, 'topic7', self.topic7_callback, 10)
        self.create_subscription(String, 'topic4', self.topic4_callback, 10)
        self.create_subscription(String, 'topic8', self.topic8_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def topic2_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def topic3_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def topic6_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def topic7_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def topic7_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def topic8_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class ROSNode4(Node):

    def __init__(self):
        self.x = 3
        super().__init__('node_4')
        self.publisher = self.create_publisher(String, 'topic1', 10)
        self.create_publisher(String, 'topic3', 10)
        self.create_subscription(String, 'topic2', self.topic2_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def topic2_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

#def main(args=None):
#    rclpy.init(args=args)

#    rclpy.shutdown()

node1 = ROSNode1()
node2 = ROSNode2()
node3 = ROSNode3()
node4 = ROSNode4()

#if __name__ == '__main__':
#    main()