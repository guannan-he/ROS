#include <ros/ros.h>
#include <action_server_client/AveragingAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>

void spinThread(){
    ros::spin();
    return;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "avgActionClient");
    actionlib::SimpleActionClient<action_server_client::AveragingAction> actionClient("avgActionServer");
    // 创建线程并开始
    boost::thread spin_thread(&spinThread);
    ROS_INFO("waiting for service");
    actionClient.waitForServer();
    ROS_INFO("server started");
    action_server_client::AveragingGoal goal;
    goal.samples = 100;
    if (argc != 1){
        goal.samples = atoi(argv[1]);
    }
    actionClient.sendGoal(goal);
    bool finsihed = actionClient.waitForResult(ros::Duration(30.0));
    if (finsihed){
        actionlib::SimpleClientGoalState state = actionClient.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else{
        ROS_WARN("timeout.");
    }
    ros::shutdown();
    // join线程
    spin_thread.join();
    return 0;
}