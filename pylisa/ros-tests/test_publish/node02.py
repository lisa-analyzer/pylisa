import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer
from std_msgs import String

class Node02(Node):
  def __init__(self):
      super().__init__("node2", namespace="ns")
      self.create_subscription(String, "topic01", 10, lambda: None)
      self.create_publisher(String, "topic02", 10)
      self.create_service(String, "service", 10, lambda: None)
      self.aC = ActionClient(self, String, "action", 10)

def main():
  node2 = Node02()

if __main__ == "main":
  main()