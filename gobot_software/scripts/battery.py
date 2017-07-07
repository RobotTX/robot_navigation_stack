#!/usr/bin/env python

import rospy
import sys 
import os

if __name__ == "__main__":
    
	rospy.init_node('battery', anonymous = True)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown(): 
		os.system("sudo python /home/gtdollar/battery/soc.py")
		rate.sleep()
		rospy.sleep(8.)
