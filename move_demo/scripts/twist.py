#!/usr/bin/env python  
#refernence: http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/  
import roslib; roslib.load_manifest('move_base')
import rospy
import tf.transformations
import os
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pow
from wheel.srv import *
from sonar.srv import *

WHEELSPEED = 0.0,0.0,0.0
LAST_TIME = time.time()
#fileHandle = open ( 'twistlog.txt', 'w' )
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getBumpersProxy = rospy.ServiceProxy( "getBumpers", GetBumpers )
flag = False

#2 is right wheel, 1 is left wheel
def setSpeed(rDirection, rSpeed, lDirection, lSpeed):
    setSpeedsProxy( str(lDirection), lSpeed, str(rDirection), rSpeed )
    #print "leftDirection and leftSpeed is :", str(lDirection), lSpeed, "rightDirection and rightSpeed is :", str(rDirection), rSpeed

def callback(msg):
    global WHEELSPEED,flag
    if not flag:
    	cmd_twist_rotation =  msg.angular.z
    	cmd_twist_x  = msg.linear.x
    	cmd_twist_y =  msg.linear.y
    	WHEELSPEED = msg.linear.x, msg.linear.y, msg.angular.z
    	bumpers = getBumpersProxy().values
    	frontBumper = bumpers[0] + bumpers[1] + bumpers[2] + bumpers[3]
    	rearBumper = bumpers[4] + bumpers[5] + bumpers[6] + bumpers[7]
    	if (frontBumper==4) and (rearBumper==4):
    	#change msg.linear.x to wheelspeed
    		if ( WHEELSPEED[0] > 0.0 ):
        		wheelSpeed = 101.9 * abs(WHEELSPEED[0]) + 0.475
    		elif ( WHEELSPEED[0] < 0.0 ):
       			wheelSpeed = 0.0 - (91.67 * abs(WHEELSPEED[0]) + 0.574)
    		else:
       			wheelSpeed = 0.0
    		#change msg.linear.z to anglespeed
    		if ( WHEELSPEED[2] != 0.0 ):
       			angleSpeed = 21.86 * abs(WHEELSPEED[2])
    		else:
       			angleSpeed = 0.0
    		if ( WHEELSPEED[2] > 0.0 ):
       			angleSpeed = angleSpeed
    		else :
       			angleSpeed = 0.0 - angleSpeed
    		#calculate rightwheelspeed and leftwheelspeed
    		rightSpeed = wheelSpeed + angleSpeed /2.0
    		leftSpeed = wheelSpeed - angleSpeed /2.0

    		if rightSpeed > 0.0 :
       			rDir = 'F'
    		else:
       			rDir = 'B'
       			rightSpeed = round(abs(rightSpeed),1)
    		if leftSpeed > 0.0 :
       			lDir = "F"
    		else:
       			lDir = "B"
       			leftSpeed = round(abs(leftSpeed),1)
    
	    	if rightSpeed < 5.0 and rightSpeed != 0.0:
       			rightSpeed = 5.0
    		if leftSpeed < 5.0 and leftSpeed != 0.0:
       			leftSpeed = 5.0
   	 	#logging = "\n rightSpeed is:" + str(rightSpeed) +" leftSpeed is:" + str(leftSpeed)
    		#fileHandle.write( logging )
    		setSpeed(rDir, rightSpeed, lDir, leftSpeed)
    	elif (frontBumper<4):
		flag = True
		setSpeed("B", 10, "B", 10)
		time.sleep(2)
		flag = False
  	elif (rearBumper<4):
		flag = True
		setSpeed("F", 10, "F", 10)
		time.sleep(2)
		flag = False

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)#/cmd_vel  
    rospy.spin()

if __name__ == '__main__':
    listener()

