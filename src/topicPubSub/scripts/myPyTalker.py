#!/usr/bin/env python
#coding=utf-8

import rospy
from topicPubSub.msg import gps
from topicPubSub.msg import timeStamp

def myPyTalker():
    rospy.init_node('pyTalkerNode')
    pub = rospy.Publisher("gpsInfo", gps, queue_size = 10)
    pubRaw = rospy.Publisher("gpsInfoRaw", gps, queue_size = 10)
    pubTime = rospy.Publisher("timeInfo", timeStamp, queue_size = 10)
    rate = rospy.Rate(1)
    state = 'working'
    x = 1.0
    y = 2.0
    while not rospy.is_shutdown():
        rospy.loginfo('Talker: x = %3.2f, y = %3.2f', x, y)
        pub.publish(gps(x, y, state))
        pubRaw.publish(gps(x, y, state))
        pubTime.publish(timeStamp("invalid time"))
        x *= 1.03
        y *= 1.03
        rate.sleep()

if __name__ == '__main__':
    myPyTalker()