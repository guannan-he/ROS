#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <string>

std::string turtleName;

// 接收到坐标指令后发布变换

void poseCallback(const turtlesim::PoseConstPtr& msg){
    static tf::TransformBroadcaster brdCst;// 变化发布器
    tf::Transform trans;// 变换
    trans.setOrigin(tf::Vector3(msg->x, msg->y, 0));
    tf::Quaternion q;// 旋转四元数
    q.setRPY(0, 0, msg->theta);
    trans.setRotation(q);
    // 发布当前乌龟相对于world的变换
    // father, son
    brdCst.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", turtleName));
    return;
}

int main(int argc, char* argv[]){
    // 不在launch文件中指定node名称，就用下面的默认名称
    ros::init(argc, argv, "broadcast_turtle");
    if(argc != 2){
        ROS_ERROR("specify name");
        return -1;
    }
    turtleName = argv[1];
    ros::NodeHandle handler;
    // 订阅当前乌龟位置
    ros::Subscriber poseSub = handler.subscribe(turtleName + "/pose", 10, &poseCallback);
    ros::spin();
    return 0;
}