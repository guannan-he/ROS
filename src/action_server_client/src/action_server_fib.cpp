#include <ros/ros.h>
// 包含动作服务器头文件
#include <actionlib/server/simple_action_server.h>
// 包含动作类型头文件
#include <action_server_client/fibonAction.h>

float calculateRate = 1.0;

class fibconAction{
    public:
    // boost::bind: 绑定函数
    // 在构造函数中创建动作服务器
    fibconAction(std::string name):
    actionName_(name), 
    actionServer_(handler_, name, boost::bind(&fibconAction::serverExec, this, _1), false)
    {
        // 开始服务
        actionServer_.start();
        ROS_INFO("%s: action server started.", actionName_.c_str());
        return;
    };
    ~fibconAction(){
        actionServer_.shutdown();
        ROS_INFO("%s: action server stopped.", actionName_.c_str());
        return;
    }
    // 动作服务器处理函数，接受包名::(action名)GoalConstPtr& 作为变量
    void serverExec(const action_server_client::fibonGoalConstPtr& goal){
        ros::Rate rate(calculateRate);
        bool succeed = true;
        // int32[] 会被转换为vector<int>
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);
        ROS_INFO("%s: executing, seq:%d, seed1 : %d, seed2: %d", actionName_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
        for(int i = 0; i < goal->order; i++){
            if(actionServer_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: preempted", actionName_.c_str());
                // 被抢占
                actionServer_.setPreempted();
                succeed = false;
                break;
            }
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i + 1]);
            // 发布反馈
            actionServer_.publishFeedback(feedback_);
            rate.sleep();
        }
        if(succeed){
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: succeed", actionName_.c_str());
            // 成功状态并发布结果
            actionServer_.setSucceeded(result_);
            // actionServer_.setAborted();
        }
        return;
    }
    protected:
    ros::NodeHandle handler_;
    // 原型：
    // actionlib::SimpleActionServer(ros::NodeHandle n, std::string name, ExecuteCallback execute_callback, bool auto_start);
    // 简单动作服务端模板函数
    actionlib::SimpleActionServer<action_server_client::fibonAction> actionServer_;
    std::string actionName_;
    action_server_client::fibonFeedback feedback_;
    action_server_client::fibonResult result_;
};



int main(int argc, char* argv[]){
    ros::init(argc, argv, "fibonServer");
    if (argc == 2){
        calculateRate = atof(argv[1]);
    }
    // 动作名称
    fibconAction fib("fibonAction");
    ros::spin();
    return 0;
}