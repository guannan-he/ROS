#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <paramDynamicSet/service_bool.h>
#include <paramDynamicSet/service_enum.h>
#include <paramDynamicSet/service_float.h>
#include <paramDynamicSet/service_int.h>
#include <paramDynamicSet/service_string.h>

bool boolCallback(paramDynamicSet::service_bool::Request& req, paramDynamicSet::service_bool::Response& resp){
    resp.retStr = req.boolVal ? "server: is True" : "server: is False";
    return true;
}

bool enumCallback(paramDynamicSet::service_enum::Request& req, paramDynamicSet::service_enum::Response& resp){
    resp.retStr = "size change received";
    return true;
}

bool floatCallback(paramDynamicSet::service_float::Request& req, paramDynamicSet::service_float::Response& resp){
    resp.retVal = -req.floatVal;
    return true;
}

bool intCallback(paramDynamicSet::service_int::Request& req, paramDynamicSet::service_int::Response& resp){
    resp.retVal = 2 * req.intVal;
    return true;
}

bool stringCallback(paramDynamicSet::service_string::Request& req, paramDynamicSet::service_string::Response& resp){
    resp.respStr = req.rqstStr;
    int left = 0, right = resp.respStr.size() - 1;
    while (left < right){
        std::swap(resp.respStr[left++], resp.respStr[right--]);
    }
    return true;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "paramServer");
    ros::NodeHandle handler;
    // 广播服务
    ros::ServiceServer boolServer = handler.advertiseService("bool_service_channel", boolCallback);
    ros::ServiceServer enumServer = handler.advertiseService("enum_service_channel", enumCallback);
    ros::ServiceServer floatServer = handler.advertiseService("float_service_channel", floatCallback);
    ros::ServiceServer intServer = handler.advertiseService("int_service_channel", intCallback);
    ros::ServiceServer stringServer = handler.advertiseService("string_service_channel", stringCallback);
    ros::spin();
    return 0;
}