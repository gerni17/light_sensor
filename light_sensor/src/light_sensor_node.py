#!/usr/bin/env python
"""
The orientation is how you write them
Plugins on raspberry pi: o o 1 o o 
                         2 3 4 o 5 
                         
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
from light_sensor.msg import LightSensorM 
import time
from Adafruit_GPIO import I2C
import RPi.GPIO as GPIO
import Adafruit_TCS34725
import smbus
import yaml
import os.path
from duckietown_utils import get_duckiefleet_root


class LightSensorNode(object):

	def __init__(self):
		#Get node name and vehicle name
		self.node_name = rospy.get_name()
		self.veh_name = self.node_name.split("/")[1]

		##GPIO setup
		#Choose BCM or BOARD numbering schemes
		GPIO.setmode(GPIO.BCM)  
		GPIO.setwarnings(False)
		GPIO.setup(18,GPIO.OUT)
		#turn off LED
		GPIO.output(18,GPIO.LOW)


		#Set integrationtime and gain
		self.tcs = Adafruit_TCS34725.TCS34725( \
				integration_time=Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_700MS, \
				gain=Adafruit_TCS34725.TCS34725_GAIN_1X)

		#Set parameter
		self.readParamFromFile()
		#Set local gain using yam
		self.gain = self.setup_parameter("gain", 0.5)

		#ROS-Publications
		self.msg_light_sensor = LightSensorM()
		self.get_lux(self.msg_light_sensor)
		self.sensor_pub = rospy.Publisher('~sensor_data', LightSensorM, queue_size=1)
		

	def get_lux(self, msg_light_sensor):
		
		count = 0
		lux = 0
		color_temp = 0

		#rate = rospy.Rate(10) # 10hz

	
		count = count + 1

		# Read R, G, B, C color data from the sensor.
		r, g, b, c = self.tcs.get_raw_data()
		# Calulate color temp
		temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
		#Calculate lux and multiply it with gain
		lux = Adafruit_TCS34725.calculate_lux(r, g, b)


		# Calculate lux out of RGB measurements.
		print("r = ", r)
		print("g = ", g)
		print("b = ", b)
		print("temp [k]= ", temp)
		print("lux = ", lux)

		# Publish to topic 
		
		# TODO: add other things to header
		msg_light_sensor.header.stamp = rospy.Time.now()
		msg_light_sensor.header.frame_id = rospy.get_namespace()[1:-1] # splicing to remove /


		msg_light_sensor.r = r
		msg_light_sensor.g = g
		msg_light_sensor.b = b
		msg_light_sensor.lux = lux
		msg_light_sensor.temp = temp
		self.sensor_pub.publish(msg_light_sensor)

		#rate.sleep()
	
	def getFilePath(self, name):
		return (get_duckiefleet_root()+'/calibrations/light-sensor/' + name + ".yaml")
    
	def readParamFromFile(self):
		#Check file existance
		fname = self.getFilePath(self.veh_name)
		# Use default.yaml if file doesn't exsit
		if not os.path.isfile(fname):
			rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
			fname = self.getFilePath("default")

		with open(fname, 'r') as in_file:
			try:
				yaml_dict = yaml.load(in_file)
			except yaml.YAMLError as exc:
				rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
				rospy.signal_shutdown()
				return

        # Set parameters using value in yaml file
		if yaml_dict is None:
        	# Empty yaml file
			return
		param_name = "gain"
		param_value = yaml_dict.get(param_name)
		if param_name is not None:
			rospy.set_param("~"+param_name, param_value)
		else:
			# Skip if not defined, use default value instead.
			pass

	def setup_parameter(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
    	# Write to parameter server for transparency
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
		return value

if __name__ == '__main__':
	rospy.init_node('light_sensor_node', anonymous=False)
	light_sensor_node = LightSensorNode()
	rospy.spin()