#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "carrot_node");
    if(argc != 2){
        ROS_ERROR("sprcify fixed or not");
        return -1;
    }
    bool fixed = (argv[1][0] == '1');
    // std::cout << fixed << std::endl;
    ros::NodeHandle handler;
    tf::TransformBroadcaster brdcstr;
    tf::Transform trans;
    ros::Rate rate(10.0);
    while (ros::ok()){
        // 发布一个胡萝卜变换，让其相对乌龟1变换
        if(fixed){
            trans.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
        }
        else{
            trans.setOrigin(tf::Vector3(2.0 * sin(ros::Time::now().toSec()), 2.0 * cos(ros::Time::now().toSec()), 0.0));
        }
        trans.setRotation(tf::Quaternion(0, 0, 0, 1));
        brdcstr.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "turtle1", "carrot1"));
        rate.sleep();
    }
    return 0;
}