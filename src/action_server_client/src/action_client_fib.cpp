#include <ros/ros.h>
#include <action_server_client/fibonAction.h>
#include <actionlib/client/simple_action_client.h>
// 定义动作的所有可能结束状态
// Simple Action Client将原始客户端状态机分为三种状态：Pending，Active & Done
#include <actionlib/client/terminal_state.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "fibonClient");
    // 简单动作客户端模板函数
    actionlib::SimpleActionClient<action_server_client::fibonAction> actionClient("fibonAction", true);
    ROS_INFO("waiting action server");
    // 必须等服务，否则会调用尚未建立的动作服务
    actionClient.waitForServer();
    action_server_client::fibonGoal goal;
    goal.order = 20;
    if (argc == 2){
        goal.order = atoi(argv[1]);
    }
    std::cout << goal.order << std::endl;
    actionClient.sendGoal(goal);
    // actionClient.cancelGoal(); //取消
    bool finishedBeforeDDL = actionClient.waitForResult(ros::Duration(30.0));
    if (finishedBeforeDDL){
        // 询问状态
        actionlib::SimpleClientGoalState state = actionClient.getState();
        ROS_INFO("action finished: %s.", state.toString().c_str());
    }
    else{
        ROS_WARN("timeout.");
    }
    return 0;
}