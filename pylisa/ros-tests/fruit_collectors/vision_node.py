import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.srv import Registration


class VisionNode(Node):

  def __init__(self):
    super().__init__('vision_node')
    self.callback_group = ReentrantCallbackGroup()

    # Listens on 'vision'
    # Performs object detection on the given image
    self.visionSubscriber = self.create_subscription(
        String,
        'vision',
        self.vision_callback,
        10,
        callback_group=self.callback_group
    )


    # Listens on 'finish'
    # Receives a message when a collector has reached a fruit
    self.finishSubscriber = self.create_subscription(
        String,
        'finish',
        self.finish_callback,
        10,
        callback_group=self.callback_group
    )


    # Publishes on 'move'
    # Publishes the destination of a given object
    self.movePublisher = self.create_publisher(
        String,
        'move',
        10,
    )

    # Service for registration
    # Registers the collector with the vision node
    self.registrationService = self.create_service(
        Registration,
        'registration',
        self.handle_register,
        callback_group=self.callback_group
    )

    self.remaining_fruits = {} # (x, y): fruit
    self.collectors = {} # id: [fruit, [x, y]]
    self.enroute = {} # id: (start, end)
    self.initialized = False


  def restart(self):
    self.initialized = False
    for id in self.collectors:
      self.get_logger().info(f'moving collector {id} to (0, 0)')
      self.enroute[id] = (self.collectors[id][1], (0, 0))
      self.movePublisher.publish(String(data=f'{id} 0 0'))
    self.get_logger().info('This task is finished, please send a new image')


  def finish_callback(self, message):
    id = int(message.data)
    self.get_logger().debug(f'finish message received: {id}')
    if id not in self.enroute:
      self.get_logger().info(f'Error: Collector {id} not in enroute')
      return
    _, end = self.enroute[id]
    if end == (0, 0):
      del self.enroute[id]
      self.get_logger().info(f'Collector {id} has reached the base')
    else:
      del self.enroute[id]
      self.collectors[id][1] = list(end)
      self.get_logger().info(f'{len(self.remaining_fruits)} fruits remaining')
      if len(self.remaining_fruits) == 0 and len(self.enroute) == 0:
        self.restart()
        return
    self.move_collector()


  def vision_callback(self, message):
    if self.initialized:
      self.get_logger().info('Please wait until the current task is finished')
      return
    output = ''
    try:
      output = subprocess.run(['python3', 'object_detector/detector/detector.py',\
                               message.data], capture_output=True)
      for line in output.stdout.decode('utf-8').split('\n'):
          if line:
            fruit, x, y = line.split(' ')
            x, y = int(x), int(y)
            self.remaining_fruits[(x, y)] = fruit
      self.get_logger().info(f'fruits detected: {message.data}')
      self.initialized = True
      self.move_collector()

    except Exception as e:
      self.get_logger().info(f'Error: {e}')


  def handle_register(self, request, response):
    self.get_logger().info('Registration request received ...')
    id, x, y = int(request.id), int(request.x), int(request.y)
    if request.action == 'Registration':
      if id in self.collectors:
        response.status = 'Error'
        response.msg = 'Collector id already registered'
        self.get_logger().info(f'Collector id {id} already registered')
        return response
      
      self.collectors[id] = [request.fruit, [x, y]]
      response.status = 'Success'
      response.msg = 'Collector registered'
      self.get_logger().info(f'Collector {id} registered')
      self.move_collector()
    else:
      response.status = 'Error'
      response.msg = 'Invalid action'
      self.get_logger().info(f'Invalid action: {request.action}')

    return response

  
  # helper - checks if two line segments intersect
  def collision_possible(self, start, end):
    def ccw(start1, end1, start2):
      return (start2[1]-start1[1]) * (end1[0]-start1[0]) > (end1[1]-start1[1]) * (start2[0]-start1[0])

    def exists_intersection(start1, end1, start2, end2):
      return ccw(start1,start2,end2) != ccw(end1,start2,end2) and ccw(start1,end1,start2) != ccw(start1,end1,end2)
    
    for route in self.enroute.values():
      if exists_intersection(start, end, route[0], route[1]):
        return True

    return False
  
  
  # helper - move as many collectors as possible to their destination
  # N*M algorithm where N is the number of collectors and M is the number of fruits
  # TODO: improve algorithm
  def move_collector(self):
    for id in self.collectors:
      if id in self.enroute:
        continue
      fruit, start = self.collectors[id]
      min_dist = float('inf')
      min_end = None
      for end in self.remaining_fruits:
        if self.remaining_fruits[end] != fruit or self.collision_possible(start, end):
          continue
        dist = (start[0]-end[0])**2 + (start[1]-end[1])**2
        if dist < min_dist:
          min_dist = dist
          min_end = end
      if min_end is not None:
        self.enroute[id] = (start, min_end)
        del self.remaining_fruits[min_end]
        self.movePublisher.publish(String(data=f'{id} {min_end[0]} {min_end[1]}'))
        self.get_logger().info(f'moving collector {id} to {min_end}')
        if not all:
          return
    


def main(args=None):
  rclpy.init(args=args)

  visionNode = VisionNode()

  executor = MultiThreadedExecutor()
  rclpy.spin(visionNode, executor)

  visionNode.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()