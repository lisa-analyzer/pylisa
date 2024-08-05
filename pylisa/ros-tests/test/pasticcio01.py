#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rclpy
from sensor_msgs.msg import Range
#TRIG IO port
TrigPin = 11  # Pin number of input terminal of ultrasonic module
#ECHO IO port
EchoPin = 8 # Pin number of output terminal of ultrasonic module

dist = 0

# S = (T2 - T1) * Vs/2
def checkdist():       #Reading distance
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TrigPin, GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.output(TrigPin, GPIO.HIGH) # Set the input end of the module to high level and emit an initial sound wave
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    while not GPIO.input(EchoPin): # When the module no longer receives the initial sound wave
        pass
    t1 = time.time() # T1 -> time when the initial sound wave is emitted
    while GPIO.input(EchoPin): # When the module receives the return sound wave
        pass
    t2 = time.time() # T2 -> time when the return sound wave is captured
    distance = (t2-t1)*340/2
    return distance


def setup():
	print('--> ULTRASONIC node')
	rclpy.init()
	#s = 14
	node = rclpy.create_node('ultrasonic_sensor', start_parameter_services=False)
	distance_pub = node.create_publisher(Range, 'range', 10)

	distance_pub_2 = node.create_publisher(Range, '~/range2', 10)
	distance_pub.publish(distance_data)
	distance_data = Range()

	#def timer_callback(): #definition of a timer function that manages all the publications
	dist = checkdist()
	#	print(dist)
	distance_data.range = dist
	#	node.get_logger().info('Publishing: "%s"' % distance_data.range)
	distance_pub.publish(distance_data)

	timer_period = 0.5  # seconds
	#timer = node.create_timer(timer_period, timer_callback)

	rclpy.spin(node)

	if KeyboardInterrupt:
		node.destroy_timer(timer)
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	setup()

#setup()
print('Shutting down: stopping ultrasonic sensor')
GPIO.cleanup()