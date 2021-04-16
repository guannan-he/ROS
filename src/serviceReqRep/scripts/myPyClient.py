#!/usr/bin/env python
# coding:utf-8

import rospy
from serviceReqRep.srv import *

def pyClient():
    rospy.init_node("pyClient")
    rospy.wait_for_service("greetingChannel")
    callSrv = rospy.ServiceProxy("greetingChannel", greeting)
    callServiceFunc(callSrv, "cock", -1)
    callServiceFunc(callSrv, "dick", 20)
    
def callServiceFunc(callSrv, name, age):
    try:
        resp = callSrv.call(name, age)
        rospy.loginfo("response from server: %s"%resp.feedback)
    except rospy.ServiceException, e:
        rospy.logwarn("service call fail: %s"%e)

if __name__ == "__main__":
    pyClient()