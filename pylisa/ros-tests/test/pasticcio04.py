import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Minimal(Node):
    def __init__(self):
        super().__init__("MINIMAL")
        p = self.create_publisher(String,"PUB",10)
        self.create_subscription(String, "SUB", self.callback, 10)
        p.publish("Hello World!")
        p.publish("Hello World 2!")
        self.create_timer(3.4, self.callback_timer)

    def callback_timer(self):
        y = self.create_publisher(String, "PUB_ON_TIMER", 10)
        y.publish("Areo")
        print("CIAO")
    def callback(self, stri):
        x = self.create_publisher(String,"PUB_ON_CALLBACK",10)
        x.publish("CIAO")
        print("CIAO")

x = Minimal()