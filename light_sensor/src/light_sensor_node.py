#!/usr/bin/env python
"""
The orientation is how you write them
Plugins on raspberry pi: o o 1 o o 
                         2 3 4 o 5 
                         
Plugins on tcs34725:     o 1 2 4 3 o 5

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


class LightSensor(object):

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
		self.mult = self.setup_parameter("~mult",1)
		self.offset = self.setup_parameter("~offset", 1)
		
		#ROS-Publications
		self.msg_light_sensor = LightSensorM()
		self.sensor_pub = rospy.Publisher('~sensor_data', LightSensorM, queue_size=1)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.get_lux()
			rate.sleep()

	def get_lux(self):
		# Read R, G, B, C color data from the sensor.
		r, g, b, c = self.tcs.get_raw_data()
		# Calulate color temp
		temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
		#Calculate lux and multiply it with gain
		lux = Adafruit_TCS34725.calculate_lux(r, g, b)
		real_lux= self.mult * lux + self.offset
		# Calculate lux out of RGB measurements.
		print("temp [k]= ", temp)
		print("r :", r)
		print("g :", g)
		print("b :", b)
		print("c :", c)
		print("lux = ", lux)
		print("real_lux: ", real_lux)

		# Publish to topic 
		
		# TODO: add other things to header
		self.msg_light_sensor.header.stamp = rospy.Time.now()
		self.msg_light_sensor.header.frame_id = rospy.get_namespace()[1:-1] # splicing to remove /


		self.msg_light_sensor.r = r
		self.msg_light_sensor.g = g
		self.msg_light_sensor.b = b
		self.msg_light_sensor.c = c
		self.msg_light_sensor.real_lux = real_lux
		self.msg_light_sensor.lux = lux
		self.msg_light_sensor.temp = temp
		self.sensor_pub.publish(self.msg_light_sensor)

		#rate.sleep()
	
	def getFilePath(self, name):
		return ('/data/config/calibrations/light-sensor/' + name + ".yaml")
    
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
		for param_name in ["mult", "offset"]:
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
	light_sensor_node = LightSensor()
	rospy.spin()