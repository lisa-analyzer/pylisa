import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer
from std_msgs import String
from std_msgs import Float64
class O:
    def __init__(self):
        self.x = 3
class MinimalActionClient(Node):
  def __init__(self):
      super().__init__("node1", namespace="ns")
      # s = String()
      # s.data = "Hello World from node1!"
      p = self.create_publisher(String, "topic01", 10)
      self.create_subscription(String, "topic03", lambda: None, 10)
      self.create_subscription(String, "topic02", lambda: None, 10)
      self.create_service(String, "service01", lambda: None)
      # p.publish(s)
      p.publish("Hello World from node1!")
      #self.create_subscription(String, "topic02", 10, lambda: None)

class Node2(Node):
  def __init__(self):
      super().__init__("node2", namespace="ns")
      p = self.create_publisher(Float64, "topic02", 10)
      self.create_subscription(String, "topic01", self.sub_callback, 10)
      self.create_client(String, "service01")
      #self.create_client(String, "service01", 10, lambda: None)
  def sub_callback(self, msg):
      x = msg * 2
      p = self.create_publisher(String, "topic03", 10)
      p.publish(x)
      #self.p.publish(x)

class Node3(Node):
  def __init__(self):
      super().__init__("node3", namespace="ns")
      self.create_subscription(String, "topic01", lambda: None, 10)
      p = self.create_publisher(Float64, "topic01", 10)
      msg = 3.14
      p.publish(msg)
      #self.create_publisher(String, "topic02", 10)
      #self.create_service(String, "service01", 10)
def main():
  node1 = MinimalActionClient()
  node2 = Node2()
  node3 = Node3()
  #node.create_publisher(String, "Publisher", 10)
  action = ActionClient(node1, String, "action")
  #serviceClient = node.create_client(String, "service", lambda:None, 10)
  nodeServer = rclpy.create_node("node4", namespace="ns")
  actionServer = ActionServer(nodeServer, String, "action", lambda: None)
o = O()
if __main__ == "main":
  main()