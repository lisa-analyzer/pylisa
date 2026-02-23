import rclpy
from rclpy.node import Node
from std_msgs import String

class Node03(Node):
  def __init__(self):
      super().__init__("node03", namespace="ns")
      self.create_subscription(String, "topic01", 10, lambda: None)
      p = self.create_publisher(String, "topic01", 10)
      msg = "Test" * 3
      p.publish(msg)

def main():
  node3 = Node03()

if __main__ == "main":
  main()