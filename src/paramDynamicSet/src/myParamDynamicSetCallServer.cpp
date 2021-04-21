#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <paramDynamicSet/service_bool.h>
#include <paramDynamicSet/service_enum.h>
#include <paramDynamicSet/service_float.h>
#include <paramDynamicSet/service_int.h>
#include <paramDynamicSet/service_string.h>


class paramCallBackServer{
    public:
    paramCallBackServer(){
        boolServer = handler.advertiseService("bool_service_channel", &paramCallBackServer::boolCallback, this);
        enumServer = handler.advertiseService("enum_service_channel", &paramCallBackServer::enumCallback, this);
        floatServer = handler.advertiseService("float_service_channel", &paramCallBackServer::floatCallback, this);
        intServer = handler.advertiseService("int_service_channel", &paramCallBackServer::intCallback, this);
        stringServer = handler.advertiseService("string_service_channel", &paramCallBackServer::stringCallback, this);
        ros::spin();
        return;
    }
    private:
    ros::NodeHandle handler;
    ros::ServiceServer boolServer;
    ros::ServiceServer enumServer;
    ros::ServiceServer floatServer;
    ros::ServiceServer intServer;
    ros::ServiceServer stringServer;
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
};



int main(int argc, char* argv[]){
    ros::init(argc, argv, "paramServer");
    paramCallBackServer service;
    return 0;
}