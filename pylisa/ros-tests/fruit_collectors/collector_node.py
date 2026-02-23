import sys
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.srv import Registration


class CollectorNode(Node):

  def __init__(self, id, fruit, speed):
    self.x = 10
    super().__init__('collector_node')
    self.callback_group = ReentrantCallbackGroup()

    #self.id, self.fruit, self.speed = id, fruit, speed
    #self.x, self.y = 0, 0


    # Publishes on 'finish'
    # Publishes a message when a collector has reached a fruit
    self.finishPublisher = self.create_publisher(
        String,
        'finish',
        10,
    )


    # Listens on 'move'
    # Moves to the destination of a given object
    self.moveSubscriber = self.create_subscription(
        String,
        'move',
        self.move_callback,
        10,
        callback_group=self.callback_group
    )


    # Service for registration
    self.registrationClient = self.create_client(
      Registration,
      'registration'
    )


    self.register()

  
  def register(self):
    while not self.registrationClient.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')
    request = Registration.Request()
    request.action = 'Registration'
    request.id = str(self.id)
    request.fruit = self.fruit
    request.x = str(self.x)
    request.y = str(self.y)
    future = self.registrationClient.call_async(request)
    future.add_done_callback(self.handle_register)


  def handle_register(self, future):
    try:
      response = future.result()
      if response.status == 'Success':
        self.get_logger().info(f'collector {self.id} registered successfully')
      elif response.msg == 'Collector id already registered':
        self.get_logger().info(f'incremeneting collector id')
        self.id += 1
        self.register()
      else:
        self.get_logger().info(f'collector {self.id} registration failed')
    
    except Exception as e:
      self.get_logger().info(f'Error in handling registration: {e}')


  def move_callback(self, message):
    id, x, y = message.data.split(' ')
    id, x, y = int(id), int(x), int(y)
    if id != self.id:
      return
    self.get_logger().info(f'collector {self.id} moving to ({x}, {y})')
    distance = ((self.x - x) ** 2 + (self.y - y) ** 2) ** 0.5
    time.sleep(distance/self.speed)
    self.x, self.y = x, y
    self.get_logger().info(f'collector {self.id} reached a destination, ' +
                           'took {:.2f} seconds'.format(distance/self.speed))
    self.finishPublisher.publish(String(data=str(self.id)))
    


def main(args=None):
  rclpy.init(args=args)

  id = sys.argv[1]
  fruit = sys.argv[2]
  speed = 50 if len(sys.argv) < 4 else sys.argv[3]
  
  collector = CollectorNode(int(id), fruit, int(speed))

  executor = MultiThreadedExecutor()
  rclpy.spin(collector, executor)

  collector.destroy_node()
  rclpy.shutdown()



if __name__ == '__main__':
  main()