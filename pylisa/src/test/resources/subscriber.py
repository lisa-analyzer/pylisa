import rclpy
from rclpy.node import Node

from std_msgs.msg import String

super().__init__('minimal_subscriber')
self.subscription = self.create_subscription(
    String,
    'my_topic',
    self.listener_callback,
10)
self.subscription  # prevent unused variable warning
