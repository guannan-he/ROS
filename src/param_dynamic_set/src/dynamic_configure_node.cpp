// 动态参数调节服务器节点
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
// 由python.cfg 文件生成
#include <param_dynamic_set/param_dynamic_set_PYConfig.h>
// 服务相关头文件
#include <param_dynamic_set/service_bool.h>
#include <param_dynamic_set/service_enum.h>
#include <param_dynamic_set/service_float.h>
#include <param_dynamic_set/service_int.h>
#include <param_dynamic_set/service_string.h>


class callbackClass
{
public:
    callbackClass(){
        handler.getParam("int_param", last.int_param);
        handler.getParam("double_param", last.double_param);
        handler.getParam("str_param", last.str_param);
        handler.getParam("bool_param", last.bool_param);
        handler.getParam("size", last.size);
        // 服务客户端初始化
        intClient = handler.serviceClient<param_dynamic_set::service_int>("int_service_channel");
        floatClient = handler.serviceClient<param_dynamic_set::service_float>("float_service_channel");
        strClient = handler.serviceClient<param_dynamic_set::service_string>("string_service_channel");
        boolClient = handler.serviceClient<param_dynamic_set::service_bool>("bool_service_channel");
        enumClient = handler.serviceClient<param_dynamic_set::service_enum>("enum_service_channel");
        // 等待服务, 必须在定义回调之前运行,否则会失败
        ROS_INFO("waiting for services");
        intClient.waitForExistence();
        floatClient.waitForExistence();
        strClient.waitForExistence();
        boolClient.waitForExistence();
        enumClient.waitForExistence();
        // https://blog.csdn.net/yaked/article/details/44942773
        // 得加上this指针才能使用, callback得用类名
        // boost::bind(&NodeExample::configCallback, node_example, _1, _2)
        // boost::bind(&类名::函数名, 类指针, _*....._*), 有几个参数使用几个占位
        // node_example -> configCallback(x, y)
        // boost::bind(&MyNode::doneCb, this, _1, _2)
        // this -> doneCb(x, y) 
        // 等效于类指针->回调函数(_*....._*)
        // bind(&X::f, &x, _1)(i);		//(&x)->f(i)
        // 第一个参数必须是可调用对象
        func = boost::bind(&callbackClass::callback, this, _1, _2);
        server.setCallback(func);
        ROS_INFO("spinning node");
        ros::spin();
        return;
    }
    void callback(param_dynamic_set::param_dynamic_set_PYConfig &config, uint32_t level){
    // ROS_INFO("%d, %f, %s, %s, %d",
    // config.int_param,
    // config.double_param,
    // config.str_param.c_str(),
    // config.bool_param ? "True" : "False",
    // config.size);
    // // 按照参数变化不用调用不同服务
    if (config.int_param != last.int_param){
        intRqst.request.intVal = config.int_param;
        if (intClient.call(intRqst)){
            ROS_INFO("intClientCall! before: %d, after: %d, callResponse: %d", last.int_param, config.int_param, intRqst.response.retVal);
        }
        else{
            ROS_WARN("int call Err");
        }
        last.int_param = config.int_param;
    }
    if (config.double_param != last.double_param){
        floatRqst.request.floatVal = config.double_param;
        if (floatClient.call(floatRqst)){
            ROS_INFO("floatClientCall! before: %f, after: %f, callResponse: %f", last.double_param, config.double_param, floatRqst.response.retVal);
        }
        else{
            ROS_WARN("float call Err");
        }
        last.double_param = config.double_param;
    }
    if (config.str_param != last.str_param){
        stringRqst.request.rqstStr = config.str_param;
        if (strClient.call(stringRqst)){
            ROS_INFO("before: \"%s\", after: \"%s\", callResponse: \"%s\"", last.str_param.c_str(), config.str_param.c_str(), stringRqst.response.respStr.c_str());
        }
        else{
            ROS_WARN("string call Err");
        }
        last.str_param = config.str_param;
    }
    if (config.bool_param != last.bool_param){
        boolRqst.request.boolVal = config.bool_param;
        if (boolClient.call(boolRqst)){
            ROS_INFO("\"%s\", after: \"%s\", callResponse: \"%s\"", last.bool_param ? "True" : "False", config.bool_param ? "True" : "False", boolRqst.response.retStr.c_str());
        }
        else{
            ROS_WARN("bool call Err");
        }
        last.bool_param = config.bool_param;
    }
    if (config.size != last.size){
        enumRqst.request.enumVal = config.size;
        if (enumClient.call(enumRqst)){
            ROS_INFO("before: %d, after: %d, callResponse: \"%s\"", last.size, config.size, enumRqst.response.retStr.c_str());
        }
        else{
            ROS_WARN("enum call Err");
        }
        last.size = config.size;
    }
    return;
}
private:
    ros::NodeHandle handler;
    param_dynamic_set::param_dynamic_set_PYConfig last;
    // 服务客户端声明
    ros::ServiceClient intClient;
    ros::ServiceClient floatClient;
    ros::ServiceClient strClient;
    ros::ServiceClient boolClient;
    ros::ServiceClient enumClient;
    // 动态参数调节服务端实例
    dynamic_reconfigure::Server<param_dynamic_set::param_dynamic_set_PYConfig> server;
    dynamic_reconfigure::Server<param_dynamic_set::param_dynamic_set_PYConfig>::CallbackType func;
    param_dynamic_set::service_int intRqst;
    param_dynamic_set::service_bool boolRqst;
    param_dynamic_set::service_float floatRqst;
    param_dynamic_set::service_string stringRqst;
    param_dynamic_set::service_enum enumRqst;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamic_configure_node");
    callbackClass configClass;
    return 0;
}