#include <ros/ros.h>
#include <string>
// #include <service_req_rep/resp.h>
// #include <service_req_rep/rqst.h>
#include <service_req_rep/greeting.h>
// #include <service_req_rep/greetingRequest.h>
// #include <service_req_rep/greetingResponse.h>

using namespace ros;

bool handleFunc(service_req_rep::greeting::Request& req, service_req_rep::greeting::Response& res){
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