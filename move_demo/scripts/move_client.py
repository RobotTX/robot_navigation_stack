#!/usr/bin/env python

import sys
import rospy
import time
import std_srvs.srv
from wheel.srv import *
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getSpeedProxy = rospy.ServiceProxy( "getSpeed", GetSpeed )
setSpeedFProxy = rospy.ServiceProxy( "setSpeedF", SetSpeedDirection )
setSpeedBProxy = rospy.ServiceProxy( "setSpeedB", SetSpeedDirection )
getEncodersProxy = rospy.ServiceProxy( "getEncoders", GetEncoders )
getEncoderProxy = rospy.ServiceProxy("getEncoder", GetEncoder )
resetEncodesProxy = rospy.ServiceProxy( "resetEncoders", std_srvs.srv.Empty ) 

def setSpeed_client(ldir, lspeed, rdir, rspeed, extime):
    try:
        #resetEncodesProxy()
        setSpeedsProxy( ldir, lspeed, rdir, rspeed )
        time.sleep(extime)
        print getSpeedProxy(1),getSpeedProxy(0)
        setSpeedsProxy( "F", 0, "F", 0 )
        encoders = getEncodersProxy().values
        print encoders[0], encoders[1]
        setSpeedsProxy( "F", 0, "F", 0 )
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [left_Direction, left_Speed, right_Direction, right_Speed, exec_time]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 6:
        ldir = str(sys.argv[1])
        lspeed = float(sys.argv[2])
        rdir = str(sys.argv[3])
        rspeed = float(sys.argv[4])
        extime = float(sys.argv[5])
        setSpeed_client(ldir, lspeed, rdir, rspeed, extime)
    else:
        print usage()
        sys.exit(1)
