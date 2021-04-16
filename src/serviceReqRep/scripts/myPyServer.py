#!/usr/bin/env python
# coding:utf-8

import rospy
from serviceReqRep.srv import *

def pyServer():
    rospy.init_node("pyServer")
    srv = rospy.Service("greetingChannel", greeting, serverResponseFunc)
    rospy.loginfo("server ready")
    rospy.spin()

def serverResponseFunc(req):
    rospy.loginfo("request from: \"%s\" with age %d", req.name, req.age)
    if req.age > -1:
        return greetingResponse("hi, \"%s\""%req.name)
    else:
        return "null"


if __name__ == "__main__":
    pyServer()