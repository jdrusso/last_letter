#!/usr/bin/env python

import rospy
import numpy
import time

from rosgraph_msgs.msg import Clock

class clock:
	def __init__(self):
		self.dt = 1.0/rospy.get_param('simRate')
		self.counter = 0
		self.time = rospy.Time.from_sec(0)
		self.pub = rospy.Publisher('clock', Clock)


if __name__=='__main__':
	try:
		rospy.init_node('clockNode')
		C = clock()
		while not rospy.is_shutdown():
			C.pub.publish(C.time)
			C.counter += 1
			C.time = rospy.Time.from_sec(C.counter*C.dt)
			time.sleep(0.1) #Does not inherit from rospy, does not obey simulation clock
			#Must be >self.dt
	except rospy.ROSInterruptException:
		pass

