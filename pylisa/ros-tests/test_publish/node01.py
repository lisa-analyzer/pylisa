import rclpy
from rclpy.node import Node
from std_msgs import String
from rclpy.action.server import ActionServer

class Node01(Node):
  def __init__(self):
      super().__init__("node1", namespace="ns")
      p = self.create_publisher(String, "topic01", 10)
      p.publish("Hello World from node1!")
      self.client = self.create_client(String, "service", 10)
      self.aS = ActionServer(self, String, "action", lambda: None)
      #self.create_subscription(String, "topic02", 10, lambda: None)

def main():
  node1 = Node01()

if __main__ == "main":
  main()