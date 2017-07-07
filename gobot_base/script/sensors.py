#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32 , Empty
from std_srvs import srv
import os
import serial
import time
import struct
import commands
from datetime import datetime 
import threading,thread
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseGoal
from math import sqrt, atan2, pow
from gobot_base.srv import *
from gobot_base.msg import *

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getSpeedProxy = rospy.ServiceProxy( "getSpeed", GetSpeed )
setSpeedFProxy = rospy.ServiceProxy( "setSpeedF", SetSpeedDirection )
setSpeedBProxy = rospy.ServiceProxy( "setSpeedB", SetSpeedDirection )

#fileHandle = open('/home/gtdollar/log/batterylog.txt','w')
goal_info = MoveBaseGoal()
twist_sign = True

bump1 = 1
bump2 = 1
bump3 = 1
bump4 = 1
bump5 = 1
bump6 = 1
bump7 = 1
bump8 = 1

value0 = 20
value1 = 20
value2 = 20
value3 = 20
value4 = 20
value5 = 20
value6 = 20

cliffDis1 = 0
cliffDis2 = 0
cliffDis3 = 0
cliffDis4 = 0

leftSignal = 0
rightSignal = 0
rearSignal = 0

rearFlag = True
distance = 10
leftFlag = False
rightFlag = False
chargingFlag = False
bumperCrashCount = 0
beginTime = time.time()
distanceIr1 = 1
distanceIr2 = 1
setChargingFlag = False
batteryStatus = 0
batteryVoltage = 0
chargingCurrent  = 0
chargingFlag = False
temperature = 0
remainCapacity = 0
fullCapacity = 0
weightInfo = 0

def get_sonar_value():
    global bump1,bump2,bump3,bump4,bump5,bump6,bump7,bump8,distanceIr1,distance2,setChargingFlag
    global value0,value1,value2,value3,value4,value5,value6
    global leftSignal, rightSignal, rearSignal,distance,twist_sign
    global batteryStatus , batteryVoltage, chargingCurrent, chargingFlag, temperature, remainCapacity, fullCapacity
    global cliffDis1, cliffDis2, cliffDis3, cliffDis4

    pub = rospy.Publisher("sonar_topic", DistanceMsg, queue_size = 10 )
    pub1= rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)
    bumperPub = rospy.Publisher("bumpers", BumperMsg, queue_size = 10 )
    irPub = rospy.Publisher("ir_signal", IrSignalMsg, queue_size = 10 )
    batteryPub = rospy.Publisher("battery_topic", BatteryInfo, queue_size = 10 )
    proximityPub = rospy.Publisher("shortSignal", ShortSignalMsg, queue_size = 10 )
    rate = rospy.Rate(30)
    writelog = False
    logtime = time.time()
    while True:
        sonar.write("\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
        nowtime = time.time()
        res = sonar.read(46)
        #print "read value lenght:", len(res)
        if len(res) == 46:
            if ord(res[0])==16:
                t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                logging = t+"{LOG]   "+hex(ord(res[0]))+" "+hex(ord(res[1]))+" "+hex(ord(res[2]))+" "+hex(ord(res[3]))+" "+hex(ord(res[4]))+" "+hex(ord(res[5]))+" "+hex(ord(res[6]))+" "+hex(ord(res[7]))+" "+hex(ord(res[8]))+" "+hex(ord(res[9]))+" "+hex(ord(res[10]))+" "+hex(ord(res[11]))+" "+hex(ord(res[12]))+" "+hex(ord(res[13]))+" "+hex(ord(res[14]))+" "+hex(ord(res[15]))+" "+hex(ord(res[16]))+" "+hex(ord(res[17]))+" "+hex(ord(res[18]))+" "+hex(ord(res[19]))+" "+hex(ord(res[20]))+" "+hex(ord(res[21]))+" "+hex(ord(res[22]))+" "+hex(ord(res[23]))+" "+hex(ord(res[24]))+" "+hex(ord(res[24]))+" "+hex(ord(res[25]))+" "+hex(ord(res[26]))+" "+hex(ord(res[27]))+" "+hex(ord(res[28]))+" "+hex(ord(res[29]))+" "+hex(ord(res[30]))+" "+hex(ord(res[31]))+" "+hex(ord(res[32]))+" "+hex(ord(res[33]))+" "+hex(ord(res[34]))+" "+hex(ord(res[35]))+" "+hex(ord(res[36]))+" "+hex(ord(res[37]))+" "+hex(ord(res[38]))+" "+hex(ord(res[39]))+" "+hex(ord(res[40]))+" "+hex(ord(res[41]))+" "+hex(ord(res[41]))+" "+hex(ord(res[42]))+" "+hex(ord(res[43]))+" "+hex(ord(res[44]))+" "+hex(ord(res[45]))+"\n"
                print logging
                if (nowtime-logtime)>300:
                    logtime=nowtime
                    #fileHandle.write(logging)
                    writelog = True
                else:
                    writelog = False
                #sonar data
                value0 = ord(res[3])*256 + ord(res[4])
                value1 = ord(res[5])*256 + ord(res[6])
                value2 = ord(res[7])*256 + ord(res[8])
                value3 = ord(res[9])*256 + ord(res[10])
                value4 = ord(res[11])*256 + ord(res[12])
                value5 = ord(res[13])*256 + ord(res[14])
                value6 = ord(res[15])*256 + ord(res[16])
                pubDist = DistanceMsg("sonar", value0, value1, value2, value3, value4, value5, value6)
                pub.publish(pubDist)
                #bumper data
                bump = bin(ord(res[17]))[2:].zfill(8)
                #print 'bump,',bump
                bump1 = int(bump[7])
                bump2 = int(bump[6])
                bump3 = int(bump[5])
                bump4 = int(bump[4])
                bump5 = int(bump[3])
                bump6 = int(bump[2])
                bump7 = int(bump[1])
                bump8 = int(bump[0])
                bumperPubDist = BumperMsg(bump1, bump2, bump3, bump4, bump5, bump6, bump7, bump8)
                bumperPub.publish(bumperPubDist)
                #distance IR data
                distanceir = bin(ord(res[21])).replace('0b','')
                if len(distanceir)==1 and distanceir=='0':
                    distanceIr1 = 0
                    distanceIr2 = 0
                elif len(distanceir)==1 and distanceir=='1':
                    distanceIr1 = 1
                    distanceIr2 = 0
                else:
                    distanceIr1 = distanceir[0]
                    distanceIr2 = distanceir[1]
                #print distanceIr1,distanceIr2
                if distanceIr1 == 0 or distanceIr2 == 0:
                    rearFlag = False
                else:
                    rearFlag = True
                proximityPubDist = ShortSignalMsg(distanceIr1, distanceIr2)
                proximityPub.publish(proximityPubDist)

                rearSignal = ord(res[18])
                leftSignal = ord(res[19])
                rightSignal = ord(res[20])
                #print rearSignal, leftSignal, rightSignal
                irPubDist = IrSignalMsg(rearSignal, leftSignal, rightSignal)
                irPub.publish(irPubDist)
                #cliff sensor data
                cliffDis1 = ord(res[22])*256 + ord(res[23])
                cliffDis2 = ord(res[24])*256 + ord(res[25])
                cliffDis3 = ord(res[26])*256 + ord(res[27])
                cliffDis4 = ord(res[28])*256 + ord(res[29])
                #print "cliff distance:", cliffDis1
                #battery data
                batteryStatus = ord(res[31])
                batteryVoltage = ord(res[32])*256 + ord(res[33])
                chargingCurrent = ord(res[34])*256 + ord(res[35])
                temperature = ord(res[36])*256 + ord(res[37])
                remainCapacity = (ord(res[38])*256 + ord(res[39]))/100
                fullCapacity = (ord(res[40])*256 + ord(res[41]))/100
                chargingFlag = chargingCurrent > 500

                batteryPubDist = BatteryInfo(batteryStatus, batteryVoltage, chargingCurrent, chargingFlag, temperature, remainCapacity, fullCapacity)
                batteryPub.publish(batteryPubDist)

                #logging = t+"[LOG]  batteryInfo soc: %d,volt: %d,chargingCurrent: %d,temp: %d,remainCapacity: %d,fullCapacity: %d \n"%(batteryInfo[0],batteryInfo[1],batteryInfo[2],batteryInfo[3],batteryInfo[4],batteryInfo[5])
                #if writelog:
                    #fileHandle.write(logging)
                #load sensor data
                weightInfo = ord(res[43])*256 + ord(res[44])



def getBumpers(req):
    global bump1,bump2,bump3,bump4,bump5,bump6,bump7,bump8
    list=[]
    list.append(bump1)
    list.append(bump2)
    list.append(bump3)
    list.append(bump4)
    list.append(bump5)
    list.append(bump6)
    list.append(bump7)
    list.append(bump8)
    return GetBumpersResponse(list) 

def getSonars(req):
    global value0,value1,value2,value3,value4,value5,value6
    list=[]
    if (value0==0.0):
        list.append(21.0)
    else:
        list.append(value0)
    if (value1==0.0):
        list.append(21.0)
    else:
        list.append(value1)
    if (value2==0.0):
        list.append(21.0)
    else:
        list.append(value2)
    if (value3==0.0):
        list.append(21.0)
    else:
        list.append(value3)
    if (value4==0.0):
        list.append(21.0)
    else:
        list.append(value4)
    if (value5==0.0):
        list.append(21.0)
    else:
        list.append(value5)
    if (value6==0.0):
        list.append(21.0)
    else:
        list.append(value6)
    return GetSonarsResponse(list)

def getBatteryInfo(req):
    global batteryStatus , batteryVoltage, chargingCurrent, chargingFlag, temperature, remainCapacity, fullCapacity
    return GetBatteryInfoResponse(batteryStatus, batteryVoltage, chargingCurrent, chargingFlag, temperature, remainCapacity, fullCapacity)

def callback(msg):
        global twist_sign,distance
        goal_info.target_pose.header.frame_id = "map"
        goal_info.target_pose.pose.position.x = 1.40
        goal_info.target_pose.pose.position.y = 5.10

        robot_rotation_z = msg.pose.pose.orientation.z
        robot_rotation_w = msg.pose.pose.orientation.w
        robot_x  = msg.pose.pose.position.x
        robot_y =  msg.pose.pose.position.y
        x = pow(robot_x-goal_info.target_pose.pose.position.x,2)
        y = pow(robot_y-goal_info.target_pose.pose.position.y,2)
        distance = sqrt(x+y)
                
def getCliffData(req):
    global cliffDis1, cliffDis2, cliffDis3, cliffDis4
    list = []
    list.append(cliffDis1)
    list.append(cliffDis2)
    list.append(cliffDis3)
    list.append(cliffDis4)
    return GetCliffDataResponse(list)

def getWeightInfo(req):
    global weightInfo
    return GetWeightInfoResponse(weightInfo)

def twist(msg):
        global twist_sign
        if (msg.linear.x==0) and (msg.linear.y==0) and (msg.angular.z==0):
                twist_sign = False
        else:
                twist_sign = True

def init():
    global sonar
    #deviceNode = "1-3.3:1.0"
    deviceNode="2-3.3:1.0"
    (status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
    devPort = "/dev"+output[output.index(deviceNode)+9:output.index(deviceNode)+17]
    sonar = serial.Serial(
            port = devPort,
            baudrate = 115200,
            stopbits = 1,
            bytesize = serial.EIGHTBITS,
            timeout = 0.5
    )
    sonar.close()
    sonar.open()

if __name__ == "__main__":
    
    rospy.init_node('sonar_pub', anonymous=False)
    init()

    rospy.Subscriber("/cmd_vel", Twist, twist)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
    bumper_service = rospy.Service('getBumpers', GetBumpers, getBumpers)
    sonar_service = rospy.Service('getSonars', GetSonars, getSonars)
    battery_service = rospy.Service('getBatteryInfo', GetBatteryInfo, getBatteryInfo)
    cliff_service = rospy.Service('getCliffData',GetCliffData,getCliffData)
    load_service = rospy.Service('getWeightInfo',GetWeightInfo,getWeightInfo)
    get_sonar_value()
    rospy.spin()


