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
import light_senso.msg 
import time
from Adafruit_GPIO import I2C
tca = I2C.get_i2c_device(address=0x70)
import RPi.GPIO as GPIO
#import xlsxwriter #only used for writing stuff to worksheet
import Adafruit_TCS34725
import smbus

GPIO.setmode(GPIO.BCM)  # choose BCM or BOARD numbering schemes
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,GPIO.LOW)#turn off LED

#ROS-Publications
rospy.init_node('light_sensor', anonymous=True, disable_rostime=False)
msg_light_sensor = light_sensor()
sensor_pub = rospy.Publisher('light_sensor', light_sensor, queue_size=50)






count = 0

# Stop the program after 10'000 measurements, just for safety.
while not rospy.is_shutdown():
	#turn off LED
	if count > 100:
		GPIO.output(18,GPIO.LOW)#turn off LED
#parameter results from sensor calibration:
	count = count + 1



	tcs = Adafruit_TCS34725.TCS34725(integration_time=Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_50MS, gain=Adafruit_TCS34725.TCS34725_GAIN_4X)

# Read R, G, B, C color data from the sensor.
	r, g, b, c = tcs.get_raw_data()
# Calulate color temp
	color_temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
# Calculate lux out of RGB measurements.
	lux = Adafruit_TCS34725.calculate_lux(r, g, b)

	print("r =", r)
	print("g =", g)
	print("b =", b)
	print("temp [k]=", color_temp)
	print("lux = ", lux)

	#Publish to topic 
	msg_light_sensor.header
	msg_light_sensor.r = msg.r
	msg_light_sensor.g = msg.g
	msg_light_sensor.b = msg.b
	msg_light_sensor.lux = msg.lux
	msg_light_sensor.temp = msg.temp
	sensor_pub.publish(msg_light_sensor)
	rospy.get_time()
	#Log everthing
	rospy.loginfomsg_light_sensor()




#except KeyboardInterrupt:
try: 
	talker()
except rospy.ROSInterruptException:
	tcs.disable()
	pass


