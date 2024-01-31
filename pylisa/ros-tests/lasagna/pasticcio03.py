import rclpy
from rclpy.node import Node
from std_msgs import String
class Minimal(Node):
    def __init__(self):
        super().__init__("MINIMAL")
        self.create_publisher(String,"PUB",10)


Minimal()