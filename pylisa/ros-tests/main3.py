import rclpy
from rclpy.node import Node

class MinimalSubscriber(rclpy.node.Node):
    def __init__(self):
        self.t = 42
        super().__init__("NODE_01")
        self.x = 10
        self.subscription = self.create_subscription(String, "TEST_TOPIC", self.listener_callback, 10)

    def listener_callback(self, msg):
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    node1 = MinimalSubscriber()

if __name__ == '__main__':
    main()