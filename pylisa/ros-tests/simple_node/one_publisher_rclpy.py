import rclpy
from std_msgs import String


def main():
  node = rclpy.create_node("node_name", namespace="main_ns")
  node.create_publisher(String, "/{node}/topic", 10)

if __main__ == "main":
  main()