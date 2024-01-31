import rclpy

def main():
  node = rclpy.create_node("Node_test")
  p = node.create_publisher(String, "publisher", 10)
  s = node.create_service(String, "service", 10)
  c = node.create_client(String, "service_client", 10, lambda: None)
if __main__ == "main":
  main()