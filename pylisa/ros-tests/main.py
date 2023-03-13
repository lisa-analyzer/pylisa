import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        #self.subscription = self.create_subscription(String, 'topic2', self.listener_callback, 20)
        #self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 30)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Test messsage'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args):
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_subscriber)
        rclpy.spin(minimal_publisher)
        minimal_subscriber.destroy_node()
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()