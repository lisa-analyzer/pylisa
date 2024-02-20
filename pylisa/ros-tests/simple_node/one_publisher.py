import rclpy
from rclpy.node import Node

class MinPub(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

MinPub()