import rclpy
#class MinimalPublisher(rclpy.node.Node):
#    def __init__(self):
#        self.x = 4
#        self.publisher = self.create_publisher(String, "TEST_TOPIC")
#        super().__init__("node_name")

class MinimalSubscriber(rclpy.node.Node):
    def __init__(self):
        super().__init__("ciao_name")
        self.x = 10
        self.subscription = self.create_subscription(String, "TEST_TOPIC")
    def test(self):
        self.a = "ciao!"
#rclpy.init()
#numpy.array([1,2])
#p = rclpy.node.Node("test_obj")
#publisher = MinimalPublisher()
#pub = publisher.create_publisher(String, "CIAO")
subscriber = MinimalSubscriber()
subscriber.test()
#sub = subscriber.create_subscription(String, "CIAO")
#n = numpy.array([1,2])
#n.reshape(1,1)
