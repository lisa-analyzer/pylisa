import rclpy
from rclpy.node import Node

super().__init__('minimal_publisher')
self.publisher_ = self.create_publisher(String, 'my_topic', 10)
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
self.i = 0
