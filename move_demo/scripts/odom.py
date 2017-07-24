#!/usr/bin/env python
# license removed for brevity
import rospy
import tf, tf2_ros

import math
import std_srvs.srv
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from wheel.srv import *

DIAMETER = 0.15   # (wheel diameter)
WHEELBASE = 0.450 # (two wheel space)
TURN = 122        # (encoders Rpm)
fileHandle = open ( 'odomlog.txt', 'w' ) 
last_rencoder = 0.0
last_lencoder = 0.0
x_x = 0.0
y_y = 0.0
th_th = 0.0
position_x = 0.0
position_y = 0.0
position_z = 0.0
#getSpeedsProxy = rospy.ServiceProxy( "getSpeeds", GetEncoders )

getSpeedProxy = rospy.ServiceProxy( "getSpeed", GetSpeed )
getEncodersProxy = rospy.ServiceProxy( "getEncoders", GetEncoders )
getEncoderProxy = rospy.ServiceProxy("getEncoder", GetEncoder )
resetEncodesProxy = rospy.ServiceProxy( "resetEncoders", std_srvs.srv.Empty )

def talker():
    global last_rencoder
    global last_lencoder
    global x_x,y_y,th_th
    global position_x,position_y,position_z
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    br =  tf2_ros.TransformBroadcaster()
    rospy.init_node('odometry_publisher')
    rate = rospy.Rate(20) # 10hz
    resetEncodesProxy()
    lastTime = rospy.Time.now()
    
    while not rospy.is_shutdown():
        # get speeds 1 is left wheel, 2 is right wheel
	ok=False
	while not ok:
		#try :
		currentTime = rospy.Time.now()
	        currentRSpeed = getSpeedProxy(0).speed
	       	currentLSpeed = getSpeedProxy(1).speed
	        encoders = getEncodersProxy().values
		ok=True
		#except:
			#print "exception"
			#ok=False
        timeDelta = (currentTime - lastTime).to_sec()

        logging = "\n speed is %d,%d"%(currentRSpeed,currentLSpeed)
        fileHandle.write( logging )

        erDis = (encoders[1]-last_rencoder)/861.4
        elDis = (encoders[0]-last_lencoder)/861.4

        rDis = erDis * DIAMETER * math.pi
        lDis = elDis * DIAMETER * math.pi

	#encoderDt = (rDis + lDis)/2
	encoderDt1 = (rDis + lDis)/2
	if ( rDis + lDis - 0.004 ) > 0.0:
		encoderDt = (rDis + lDis - 0.004)/2
	else:
		encoderDt = (rDis + lDis)/2
        encoderth = (rDis - lDis)/WHEELBASE 

        '''
        if encoderth != 0.0:
        	encoderDt = (encoderDs * 2 * math.sin(encoderth/2))/encoderth
        else:
		encoderDt = encoderDs * 2 * math.sin(encoderth/2)
        '''
        #print encoderDs,encoderDt
	if (th_th==0) and (encoderth!=0):
		th_th = encoderth
	#if encoderDt != 0:
		#encoder_x = (encoderDt-0.0005) * math.cos(encoderth/2 + th_th)
	#else:
		#encoder_x = encoderDt * math.cos(encoderth/2 + th_th)
	encoder_x = encoderDt * math.cos(encoderth/2 + th_th)
	encoder_y = encoderDt1 * math.sin(encoderth/2 + th_th)

        Vx = encoderDt1/timeDelta
        Vy = 0.0
        Vth = encoderth/timeDelta
   
        last_rencoder = encoders[1]
        last_lencoder = encoders[0]

	if encoder_x!=0:
		encoder_x = encoder_x + 0.0001
        x_x = x_x + encoder_x 
        y_y = y_y + encoder_y

        th_th = th_th + encoderth
        logging = "\n odometry x,y,th is : %f,%f,%f,%f,%f,%f" %(x_x,y_y,th_th,encoder_x,encoder_y,encoderth)
        fileHandle.write ( logging ) 

        logging = "\n hector position x,y,th is : %f,%f,%f" %(position_x, position_y, position_z)
        fileHandle.write ( logging )

        quat = tf.transformations.quaternion_from_euler(0,0,th_th)
        
        odomMsg = Odometry()
        odomTrans = TransformStamped()

        odomTrans.header.stamp = currentTime
        odomTrans.header.frame_id = "/odom"
        odomTrans.child_frame_id = "/base_link"
        odomTrans.transform.translation.x = x_x
        odomTrans.transform.translation.y = y_y
        odomTrans.transform.translation.z = 0.0
        odomTrans.transform.rotation.x = quat[0]
        odomTrans.transform.rotation.y = quat[1]
        odomTrans.transform.rotation.z = quat[2]
        odomTrans.transform.rotation.w = quat[3]
        br.sendTransform( odomTrans)

        odomMsg.header.stamp = currentTime
        odomMsg.header.frame_id = "odom"
        odomMsg.child_frame_id = "base_link"
        odomMsg.pose.pose.position.x = x_x
        odomMsg.pose.pose.position.y = y_y
        odomMsg.pose.pose.position.z = 0.0
        odomMsg.pose.pose.orientation.x = quat[0]
        odomMsg.pose.pose.orientation.y = quat[1]
        odomMsg.pose.pose.orientation.z = quat[2]
        odomMsg.pose.pose.orientation.w = quat[3]
        
        odomMsg.twist.twist.linear.x = Vx
        odomMsg.twist.twist.linear.y = Vy 
        odomMsg.twist.twist.angular.z = Vth
        
        pub.publish(odomMsg)

        lastTime = currentTime
        rate.sleep()
    fileHandle.close()

def callback(msg):
        global position_x,position_y,position_z
        position_x = msg.pose.position.x
        position_y = msg.pose.position.y
        position_z = msg.pose.orientation.z

if __name__ == '__main__':
    try:
        talker()
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)
    except rospy.ROSInterruptException:
        pass
