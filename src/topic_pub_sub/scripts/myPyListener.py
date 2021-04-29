#!/usr/bin/env python
#coding=utf-8

import rospy
import math
import time
from topic_pub_sub.msg import gps
from topic_pub_sub.msg import timeStamp

def gpsCallback(msg):
    dist = math.sqrt(math.pow(msg.x, 2) + math.pow(msg.y, 2))
    rospy.loginfo("Listener: distanct = %3.2f, state: %s", dist, msg.state)

def gpsRawCallback(rawMsg):
    rospy.loginfo("GPS raw info: x = %3.2f, y = %3.2f, state: %s", rawMsg.x, rawMsg.y, rawMsg.state)
    
def timeCallback(timeStamp):
    rospy.loginfo("%s, %s", timeStamp.time, time.time())

def myPyListener():
    rospy.init_node("myPyListener")
    sub = rospy.Subscriber("gpsInfo", gps, gpsCallback)
    subRaw = rospy.Subscriber("gpsInfoRaw", gps, gpsRawCallback)
    subTime = rospy.Subscriber("timeInfo", timeStamp, timeCallback)
    rospy.spin()

if __name__ == '__main__':
    myPyListener()