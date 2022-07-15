#!/usr/bin/python

#MIT License

#Copyright (c) [year] [fullname]

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import rospy
from altimeter.BMP180_driver import BMP180
from altimeter.msg import Altimeter

class AltimeterBMP180():

	def __init__(self):
		self.pub_alt = rospy.Publisher("~BMP180",
			Altimeter, queue_size = 10)
		self.alt_sensor = BMP180()

		self.temp = 0.0	# In celcius degrees
		self.alt = 0.0 # In meters


		self.alt_dat_seq_counter = 0

	def read_data(self):

		self.temp = self.alt_sensor.temperature()
		self.alt = self.alt_sensor.altitude()
	

	def publish_data(self):

		alt_data = Altimeter()

		alt_data.header.seq = self.alt_dat_seq_counter
		alt_data.header.stamp = rospy.Time.now()
		alt_data.header.frame_id = ''

		alt_data.altitude = self.alt
		alt_data.temperature = self.temp

		self.pub_alt.publish(alt_data)

if __name__ == '__main__':
	
	# Init node
	rospy.init_node("altimeter")

	altimeter = AltimeterBMP180()

	rate = rospy.Rate(2)

	while not rospy.is_shutdown():
		altimeter.read_data()
		altimeter.publish_data()
		rate.sleep()
