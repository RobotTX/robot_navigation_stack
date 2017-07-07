#!/usr/bin/env python

import rospy
import std_srvs
import commands
from std_srvs import srv
import rospkg
from std_msgs.msg import String,Empty
import serial
import time
import struct
from gobot_base.srv import *

testEnd=True

def setSpeedAction(wheel,direction,percent):
	global testEnd
        #wheel= req.wheel
        #direction = req.direction
        #percent = req.velocity
        if direction == 'F' :
                speed = 128 - ((percent * 127) / 100)
        else :
                speed = 128 + ((percent * 128 ) / 100 )

        if wheel == 1 :
                wheel_serial.write(serial.to_bytes([0x00,0x31 ]))
        else :
                wheel_serial.write(serial.to_bytes([0x00,0x32 ]))
        wheel_serial.write(chr(speed))

def setSpeeds(req):
	global testEnd
	while not testEnd :
        	continue
    	testEnd=False
    	setSpeedAction(1,req.directionR,req.velocityR)
        setSpeedAction(2,req.directionL,req.velocityL)
    	testEnd=True
        return SetSpeedsResponse()

def setSpeedF(req):
        global testEnd

        while not testEnd :
                continue
        testEnd=False
        speed = 128 + ((req.velocity * 127) / 100)
        if req.wheel == 1 :
                wheel_serial.write(serial.to_bytes([0x00,0x31 ]))
        else :
                wheel_serial.write(serial.to_bytes([0x00,0x32 ]))
        wheel_serial.write(chr(speed))
        testEnd=True
        return SetSpeedDirectionResponse()

def setSpeedB(req):
        global testEnd

        while not testEnd :
                continue
        testEnd=False

        speed = 128 - ((req.velocity * 127) / 100)
        if req.wheel == 1 :
                wheel_serial.write(serial.to_bytes([0x00,0x31 ]))
        else :
                wheel_serial.write(serial.to_bytes([0x00,0x32 ]))
        wheel_serial.write(chr(speed))
        testEnd=True
        return SetSpeedDirectionResponse()

	# wheel : 1 = right ; 2 = left
def setSpeed(req):
        global testEnd

        while not testEnd :
                continue
        testEnd=False
	setSpeedAction(req.wheel,req.direction,req.velocity)
        testEnd=True
	return SetSpeedResponse()


def getSpeed(req) :
        global testEnd
        print "into getSpeed, wheel is :", req.wheel
        while not testEnd :
                continue
        testEnd=False
    	wheel = req.wheel
        if wheel == 1 :
                wheel_serial.write(serial.to_bytes([0x00,0x21]))
        else :
                wheel_serial.write(serial.to_bytes([0x00,0x22]))
        res= wheel_serial.read()
        res= ord(res)
        if res < 128 :
                res = (128 - res) * 100 / 128
                res = 0 - res
        else :
                res = (res - 128) * 100 / 127
        testEnd=True
        print "getSpeed finished"
        return GetSpeedResponse(-res)



def calculate(bytes):
    reverse = False
    if str( bytes[0].encode('hex') ) == 'ff' :
        reverse = True

    hex = "0x"
    for i in range(0,4) :
        hex = hex + str( bytes[i].encode('hex') )
    testEnd=True

    if reverse :
        max = 4294967295
        return  int(hex, 16) - max
    else:
        return  int(hex,16)



def getEncoder(req) :
        global testEnd
        while not testEnd :
                continue
        testEnd=False
        wheel = req.wheel

        if wheel == 1 :
                wheel_serial.write( serial.to_bytes( [0x00,0x23] ) )
        else :
                wheel_serial.write( serial.to_bytes( [0x00,0x24] ) )
        res= wheel_serial.read( 4 )
        val = calculate(res)
        testEnd=True
        return GetEncoderResponse( val )

def getEncoders(req) :
    global testEnd
    while not testEnd :
        continue
    testEnd=False
    wheel_serial.write( serial.to_bytes( [0x00,0x25] ) )
    res1=wheel_serial.read(4)
    res2=wheel_serial.read(4)
    testEnd=True
    return GetEncodersResponse(calculate(res1), calculate(res2))

def testEncoders(req) :
    global testEnd

    print "ok"
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        while not testEnd :
            continue
        testEnd=False
        wheel_serial.write( serial.to_bytes( [0x00,0x25] ) )
        res1=wheel_serial.read(4)
        res2=wheel_serial.read(4)
        print "result : %d and %d " % (calculate(res1), calculate(res2))
        testEnd=True
    return True

def resetEncoders(req) :
        global testEnd

        while not testEnd :
                continue
        testEnd=False
    	wheel_serial.write(serial.to_bytes([0x00,0x35]))
        testEnd=True
    	return std_srvs.srv.EmptyResponse()

def init():
    	global wheel_serial,testEnd
	#deviceNode = "1-3.3:1.1"
	deviceNode="2-3.3:1.1"
	(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
	devPort = "/dev"+output[output.index(deviceNode)+9:output.index(deviceNode)+17]
	print devPort
        wheel_serial = serial.Serial(port = devPort, baudrate=9600)
        wheel_serial.close()
        wheel_serial.open()

    	#wheel mode 0
        wheel_serial.write(serial.to_bytes([0x00,0x34,0x00]))
        #disable 2sec timeout
        wheel_serial.write(serial.to_bytes([0x00,0x38]))
        #acceleration step 5
        wheel_serial.write(serial.to_bytes([0x00,0x33,0x01]))


if __name__ == "__main__":
    rospy.init_node('MD49_listener')
    init()
    s = rospy.Service('setSpeed', SetSpeed, setSpeed)
    s = rospy.Service('getSpeed', GetSpeed, getSpeed)
    s = rospy.Service('getEncoder', GetEncoder, getEncoder)
    s = rospy.Service('getEncoders', GetEncoders, getEncoders)
    s = rospy.Service('resetEncoders', std_srvs.srv.Empty, resetEncoders)
    s = rospy.Service('setSpeedF', SetSpeedDirection, setSpeedF)
    s = rospy.Service('setSpeedB', SetSpeedDirection, setSpeedB)
    s = rospy.Service('setSpeeds', SetSpeeds, setSpeeds)
    s = rospy.Service('testEncoders', std_srvs.srv.Empty, testEncoders)

    rospy.spin()


