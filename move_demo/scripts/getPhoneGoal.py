#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import time
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

def getGoal():
	pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10 )
	nav_goal = PoseStamped()
	while True:
		res = phoneSign.read(1)
		phoneSignNum = ord(res)
		if phoneSignNum == 1:
			nav_goal.header.frame_id = "map"
			nav_goal.pose.position.x = 6.0
			nav_goal.pose.position.y = 1.0
			nav_goal.pose.position.z = 0.0
			nav_goal.pose.orientation.z = -0.3988
			nav_goal.pose.orientation.w = 0.917
		elif phoneSignNum == 3:
			nav_goal.header.frame_id = "map"
                        nav_goal.pose.position.x = -6.4
                        nav_goal.pose.position.y = -1.1
                        nav_goal.pose.position.z = 0.0
                        nav_goal.pose.orientation.z = -0.448
			nav_goal.pose.orientation.w = 0.894
		elif phoneSignNum == 2:
			nav_goal.header.frame_id = "map"
                        nav_goal.pose.position.x = 0.0
                        nav_goal.pose.position.y = 0.0
                        nav_goal.pose.position.z = 0.0
                        nav_goal.pose.orientation.z = 0.0
			nav_goal.pose.orientation.w = 1.0
		elif phoneSignNum == 4:
			nav_goal.header.frame_id = "map"
                        nav_goal.pose.position.x = -5.77
                        nav_goal.pose.position.y = 8.015
                        nav_goal.pose.position.z = 0.0
                        nav_goal.pose.orientation.z = 0.75
			nav_goal.pose.orientation.w = 0.66
		elif phoneSignNum == 5:
			nav_goal.header.frame_id = "map"
			nav_goal.pose.position.x = -7.5
			nav_goal.pose.position.y = -1.1
			nav_goal.pose.position.z = 0.0
			nav_goal.pose.orientation.z = -0.9
			nav_goal.pose.orientation.w = 0.3
		pub.publish(nav_goal)

def getRobotStatus(status):
	print status

def init():
	global phoneSign
	phoneSign = serial.Serial(port = "/dev/ttyS0", baudrate=115200)
	phoneSign.close()
	phoneSign.open()

if __name__ == '__main__':
	try:
		rospy.init_node('goal_pub', anonymous=False)
        	init()	
		getGoal()
		rospy.Subscriber("/move_base/status", GoalStatus, getRobotStatus)
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
