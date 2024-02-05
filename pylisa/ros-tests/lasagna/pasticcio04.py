import rclpy
from rclpy.node import Node
from std_msgs import String
class Minimal(Node):
    def __init__(self):
        super().__init__("MINIMAL")
        p = self.create_publisher(String,"PUB",10)
        self.create_subscription(String, "SUB", self.callback, 10)
        p.publish("Hello World!")
    def callback(self, stri):
        self.create_publisher(String,"PUB_ON_CALLBACK",10)
        print("CIAO")
Minimal()