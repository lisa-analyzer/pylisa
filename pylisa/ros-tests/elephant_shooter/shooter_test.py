import rclpy
import numpy as np
from rclpy.node import Node
import yaml
## This node is created to publish the data to store in the yaml file in order to fixed the parameter for the future use
## In which the parameter will be come from the testing file and used the data from yaml file to use in the automatic shooter
from std_msgs.msg import UInt16 # laser: subscribe the data from laser
from std_msgs.msg import UInt8 # motor: publish the data to the motor 
from std_msgs.msg import Int8 # button: to control shoot or not shoot
from std_msgs.msg import Float32 # Parameter publisher
from std_msgs.msg import Float32MultiArray

from time import sleep

from shooter.shooter_test import *

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.adjust_sub = self.create_subscription(Float32, "adjust", self.adjust_callback,10)
        self.shooter_pub = self.create_publisher(UInt16, 'shooter', 10)
        self.param_pub = self.create_publisher(Float32MultiArray, 'stored', 10)
        self.shoot_pub = self.create_publisher(UInt8, 'process_state', 10)


        self.button_command = 0
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0
        self.adjust = 0.0
        self.xin = 0.0
        self.a = np.zeros(25)
        self.data = [self.a[0], self.a[1],self.a[2], self.a[3], self.a[4],
                                    self.a[5], self.a[6],self.a[7], self.a[8], self.a[9],
                                    self.a[10], self.a[11],self.a[12], self.a[13], self.a[14],
                                    self.a[15], self.a[16],self.a[17], self.a[18], self.a[19],
                                    self.a[20], self.a[21],self.a[22], self.a[23], self.a[24],
                                    ]
    
    def adjust_callback(self, adjust_msg):
        self.adjust = adjust_msg.data

    def button_callback(self, button_msg):
        button_command = int(button_msg.data)
    
        while(button_command == 1):
            self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
            distance = (4.439 - 0.765)/(3495 - 6)*(self.laser_data - 6) + 0.765
            self.rps, self.xin, self.a = shooter(distance, self.adjust, self.a).shooter()

            if(self.rps == 7433):
                self.rps = 0 

            print(self.xin, distance)
            print(self.a)
            param_msg = Float32MultiArray()
            param_msg.data = [self.a[0], self.a[1],self.a[2], self.a[3], self.a[4],
                                    self.a[5], self.a[6],self.a[7], self.a[8], self.a[9],
                                    self.a[10], self.a[11],self.a[12], self.a[13], self.a[14],
                                    self.a[15], self.a[16],self.a[17], self.a[18], self.a[19],
                                    self.a[20], self.a[21],self.a[22], self.a[23], self.a[24],
                                    ]
            #param_msg.data = self.a
            self.param_pub.publish(param_msg)
            shooter_msg = UInt16()
            shooter_msg.data = self.rps
            self.shooter_pub.publish(shooter_msg)
            ## here
            shoot_msg = UInt8()
            shoot_msg.data = 2
            self.shoot_pub.publish(shoot_msg)
            self.rps = 0
            break

        

    def laser_callback(self, laser_msg):
        self.laser_data = laser_msg.data

 
        

def main(args=None):
    rclpy.init(args=args)
    shooter_node = ShooterNode()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()

