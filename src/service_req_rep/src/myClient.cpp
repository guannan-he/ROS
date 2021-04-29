#include <ros/ros.h>
#include <service_req_rep/greeting.h>

using namespace ros;
void callService(ServiceClient& client, service_req_rep::greeting& srvRqst){
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
    ServiceClient client = handler.serviceClient<service_req_rep::greeting>("greetingChannel");
    service_req_rep::greeting srvRqstValid, srvRqstInValid;
    srvRqstValid.request.name = "pinky";
    srvRqstValid.request.age = 20;
    srvRqstInValid.request.name = "purky";
    srvRqstInValid.request.age = -1;
    service_req_rep::greeting& srvRqst = srvRqstValid;
    callService(client, srvRqst);
    srvRqst = srvRqstInValid;
    callService(client, srvRqst);
    ros::spin();
    return 0;
}