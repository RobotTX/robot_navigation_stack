#!/usr/bin/env python  
import roslib; roslib.load_manifest('move_base')
import rospy
import  os
import  sys
import  tty, termios
import time
import std_srvs.srv
from wheel.srv import *

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
resetEncodesProxy = rospy.ServiceProxy( "resetEncoders", std_srvs.srv.Empty )
getEncodersProxy = rospy.ServiceProxy( "getEncoders", GetEncoders )

if __name__ == '__main__':
    resetEncodesProxy()
    print "Reading form keybord"
    print """ a w s d z q  """
    print 'press Q to quit'
    while True:
        fd=sys.stdin.fileno()
        old_settings=termios.tcgetattr(fd)
        #old_settings[3]= old_settings[3] & ~termios.ICANON & ~termios.ECHO    
        try:
            tty.setraw(fd)
            ch=sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            #print 'error'  
        if ch=='w':
            	print 'move forward'
		setSpeedsProxy("F", 20, "F", 20)
        elif ch=='s':
            	print 'move back'
		setSpeedsProxy("B", 20, "B", 20)
        elif ch=='a':
            	print "turn left!"
		setSpeedsProxy("B", 10, "F", 10)
        elif ch=='d':
            	print "turn right!"
		setSpeedsProxy("F", 10, "B", 10)
        elif ch=='z':
            	print "stop motor!"
		setSpeedsProxy("F", 0, "F", 0)
        elif ch=='q':
            print "shutdown!"
            break
	elif ch=='n':
		encoders = getEncodersProxy().values
        	print time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),encoders[0], encoders[1]
        elif ord(ch)==0x3:
            print "shutdown"
            break
        print "Reading form keybord"
        print """   i j  k  l m"""
        print 'press Q or ctrl+c to quit'
