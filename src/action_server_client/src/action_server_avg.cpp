#include <ros/ros.h>
#include <action_server_client/AveragingAction.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float32.h>

class avgActionServer{
    public:
    // 只用handler, 名称, 起始状态初始化_actionServer
    avgActionServer(std::string name):
    _actionName(name), 
    _actionServer(_handler, _actionName, false){
        // 注册抢占与目标回调函数
        _actionServer.registerGoalCallback(boost::bind(&avgActionServer::actionCallbackGoal, this));
        _actionServer.registerPreemptCallback(boost::bind(&avgActionServer::actionCallbackPreempt, this));
        // 注册订阅
        _sub = _handler.subscribe("/randomNumber", 1, &avgActionServer::analysisCallback, this);
        _actionServer.start();
        ROS_INFO("avg_action_server started.");
        return;
    }
    void actionCallbackGoal(){
        _dataCnt = 0;
        _sum = 0;
        _sumSq = 0;
        // 没有传入参数，相反直接从服务器获取目标
        _goal = _actionServer.acceptNewGoal()->samples;
        return;
    }
    void actionCallbackPreempt(){// 事件触发
        ROS_INFO("%s: preempted", _actionName.c_str());
        _actionServer.setPreempted();
        return;
    }
    void analysisCallback(const std_msgs::Float32::ConstPtr& msg){
        if(!_actionServer.isActive()){// 确保服务器接收到目标后才处理数据
            return;
        }
        _dataCnt++;
        _feedback.data = msg->data;
        _feedback.sample = _dataCnt;
        _sum += msg->data;
        _feedback.avg = _sum / _dataCnt;
        _sumSq += pow(msg->data, 2);
        _feedback.std_dev = sqrt(fabs(_sumSq / _dataCnt) - pow(_feedback.avg, 2));
        _actionServer.publishFeedback(_feedback);
        if (_dataCnt == _goal){
            _result.avg = _feedback.avg;
            _result.std_dev = _feedback.std_dev;
            if(_result.avg < 5.0){
                ROS_WARN("%s: aborted", _actionName.c_str());
                // 同样发布结果，但是结束状态是aborted
                // 会将服务器失效
                _actionServer.setAborted(_result);
            }
            else{
                ROS_INFO("%s: succeed", _actionName.c_str());
                // 会将服务器失效
                _actionServer.setSucceeded(_result);
            }
        }
        return;
    }
    protected:
    ros::NodeHandle _handler;
    std::string _actionName;
    actionlib::SimpleActionServer<action_server_client::AveragingAction> _actionServer;
    action_server_client::AveragingFeedback _feedback;
    action_server_client::AveragingResult _result;
    int _dataCnt, _goal;
    float _sum, _sumSq;
    ros::Subscriber _sub;
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "avgActionServer");
    avgActionServer actionServer(ros::this_node::getName());
    ros::spin();
    return 0;
}