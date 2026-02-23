import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String

class N(Node):
	def __init__(self):
		super().__init__('NODE')
		self.publisher = self.create_publisher(String, 'topic', 10)

#x = 3
node = N()

