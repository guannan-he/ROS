#include <ros/ros.h>
#include <string>
// #include <serviceReqRep/resp.h>
// #include <serviceReqRep/rqst.h>
#include <serviceReqRep/greeting.h>
// #include <serviceReqRep/greetingRequest.h>
// #include <serviceReqRep/greetingResponse.h>

using namespace ros;

bool handleFunc(serviceReqRep::greeting::Request& req, serviceReqRep::greeting::Response& res){
    ROS_INFO("Requst from %s with age %d.", req.name.c_str(), req.age);
    res.feedback = "Hi! " + req.name + "!";
    return req.age > -1;
}

int main(int argc, char* argv[]){
    init(argc, argv, "myServer");
    NodeHandle handler;
    // 定义服务时不需要指定服务类型
    ServiceServer nodeService = handler.advertiseService("greetingChannel", handleFunc);
    spin();
    return 0;
}