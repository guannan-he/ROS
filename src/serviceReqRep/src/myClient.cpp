#include <ros/ros.h>
#include <serviceReqRep/greeting.h>

using namespace ros;
void callService(ServiceClient& client, serviceReqRep::greeting& srvRqst){
    if (client.call(srvRqst)){
        ROS_INFO("response from server: %s", srvRqst.response.feedback.c_str());
    }
    else{
        ROS_WARN("invalid age");
    }
    return;
}
int main(int argc, char* argv[]){
    init(argc, argv, "myClient");
    NodeHandle handler;
    // 定义请求客户端时需要指定服务类型
    ServiceClient client = handler.serviceClient<serviceReqRep::greeting>("greetingChannel");
    serviceReqRep::greeting srvRqstValid, srvRqstInValid;
    srvRqstValid.request.name = "cock";
    srvRqstValid.request.age = 20;
    srvRqstInValid.request.name = "dick";
    srvRqstInValid.request.age = -1;
    serviceReqRep::greeting& srvRqst = srvRqstValid;
    callService(client, srvRqst);
    srvRqst = srvRqstInValid;
    callService(client, srvRqst);
    return 0;
}