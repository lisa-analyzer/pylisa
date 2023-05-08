import rclpy
from rclpy.node import Node

x = 3 + 5

class MinimalPublisher(rclpy.node.Node):
    def __init__(self):
        self.x = 4
        self.publisher = self.create_publisher(String, "TEST_TOPIC")
        #self.publisher.publish("message")
x2 = MinimalPublisher()
p = x2.create_publisher(String, "CIAO")
c = x2.x
cx = x2.publisher
#x4 = x2.publisher
#x4.publish("msg")

n = numpy.array([1,2])
n.reshape(1,1)