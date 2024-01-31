import rclpy
from rclpy.node import Node
from std_msgs import String
class Pub(Node):
  def __init__(self, nodeName = "test_publisher", topicName = "topic"):
    super().__init__(nodeName)
    self.publisher = self.create_publisher(String, topicName, 10)
    self.subscription = self.create_subscription(String, topicName, 10, lambda : None)

def main():
  node = Pub("pub_cp", "topic_cp")

if __name__ == "main":
  main()