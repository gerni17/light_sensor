#!/usr/bin/env python
"""
The orientation is how you write them
Plugins on raspberry pi: o o 1 o o 5
                         2 3 4 o o o
                         
Plugins on tcs34725:     o 1 2 4 3 o 5

run with python3 light_sensor.py

Troubleshooting:
if no package imported try: 'pip3 install Adafruit_TCS34725'


Adjusts the gain on the TCS34725 (adjusts the sensitivity to light).
Use one of the following constants:
 - TCS34725_GAIN_1X   = No gain
 - TCS34725_GAIN_4X   = 2x gain
 - TCS34725_GAIN_16X  = 16x gain
 - TCS34725_GAIN_60X  = 60x gain

"""
import rospy
from light_sensor.msg import LightSensor 
import time
from Adafruit_GPIO import I2C
import RPi.GPIO as GPIO
import Adafruit_TCS34725
import smbus

GPIO.setmode(GPIO.BCM)  # choose BCM or BOARD numbering schemes
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,GPIO.LOW)#turn off LED

#ROS-Publications
rospy.init_node('light_sensor_node', anonymous=False)
msg_light_sensor = LightSensor()
sensor_pub = rospy.Publisher('sensor_data', LightSensor, queue_size=1)

<<<<<<< HEAD:light_sensor/src/light_sensor.py
=======
#parameter results from sensor calibration:
#deslux = [0,0,138.6,166.6,95.4,204.4,163.7,200.6]
>>>>>>> 38b6f29a3fa84de8cb9e143aeaa0bd9ec4d286ae:light_sensor/src/light_sensor_node.py

count = 0
tcs = Adafruit_TCS34725.TCS34725(integration_time=Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_50MS, gain=Adafruit_TCS34725.TCS34725_GAIN_4X)

r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
	#turn off LED
	if count > 100:
		GPIO.output(18,GPIO.LOW)#turn off LED
	count = count + 1

	# Read R, G, B, C color data from the sensor.
	r, g, b, c = tcs.get_raw_data()
	# Calulate color temp
	color_temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
	# Calculate lux out of RGB measurements.
	lux = Adafruit_TCS34725.calculate_lux(r, g, b)

	print("r = ", r)
	print("g = ", g)
	print("b = ", b)
	print("temp [k]= ", color_temp)
	print("lux = ", lux)

	# Publish to topic 
	
	# TODO: add other things to header
	msg_light_sensor.header.stamp = rospy.Time.now()

	msg_light_sensor.r = r
	msg_light_sensor.g = g
	msg_light_sensor.b = b
	msg_light_sensor.lux = lux
	msg_light_sensor.temp = temp

	sensor_pub.publish(msg_light_sensor)

<<<<<<< HEAD:light_sensor/src/light_sensor.py
#except KeyboardInterrupt:
try: 
	talker()
except rospy.ROSInterruptException:
	tcs.disable()
	pass


=======
	r.sleep()

# Disable sensor
tcs.disable()
>>>>>>> 38b6f29a3fa84de8cb9e143aeaa0bd9ec4d286ae:light_sensor/src/light_sensor_node.py
