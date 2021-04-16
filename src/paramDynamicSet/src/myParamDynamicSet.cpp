#include <ros/ros.h>

using namespace ros;

int main(int argc, char* argv[]){
    init(argc, argv, "paramServer");
    NodeHandle handler;
    int param1 = 10, param2 = 20, param3 = 30, param4 = 40, param5 = 50;
    // 三种获取参数方法，key-value
    param::get("param1", param1);
    handler.getParam("param2", param2);
    handler.param("param3", param3, 123);// 第三个是未找到时的默认值
    // 设置参数
    handler.setParam("param4", param4);
    param::set("param5", param5);
    // 查询参数是否存在
    handler.hasParam("param6");
    param::has("param7");
    // 删除参数
    handler.deleteParam("param8");
    param::del("param9");
    return 0;
}