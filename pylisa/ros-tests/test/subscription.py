# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from std_msgs.msg import String
from rclpy.qos import QoSProfile

def main(args=None):
    rclpy.init(args=args)
    QOS_RKL10V = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=qos_depth,
        durability=QoSDurabilityPolicy.VOLATILE,
        avoid_ros_namespace_conventions =  True
    )
    node = rclpy.create_node('minimal_subscriber')
    #publisher = node.create_publisher(String, 'topic', 10)
    x = 30
    subscriber = node.create_subscription(String, 'topic', lambda: None, QOS_RKL10V)
    i = 0
    node2 = rclpy.create_node('minimal_publisher')
    publisher = node2.create_publisher(String, 'topic', QOS_RKL10V)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
