import rclpy
from rclpy.node import Node

class N1(Node):
    def __init__(self):
        self.x = 3
        super().__init__("node_1")
        self.create_publisher(String, "topic1", 10)
    def listener_callback(self, msg):
        print(msg)

class N2(Node):
    def __init__(self):
        self.x = 3
        super().__init__("node_2")
        self.create_subscription(String, "topic1", self.listener_callback, 10)
        self.create_publisher(String, "topic2", 10)
    def listener_callback(self, msg):
        print(msg)
class N3(Node):
    def __init__(self):
        self.x = 3
        super().__init__("node_3")
        self.create_subscription(String, "topic2", self.listener_callback, 10)
    def listener_callback(self, msg):
        print(msg)
def main(args=None):
    rclpy.init(args=args)
    node1 = N1()
    NODE2 = N2()
    NODE3 = N3()

if __name__ == '__main__':
    main()