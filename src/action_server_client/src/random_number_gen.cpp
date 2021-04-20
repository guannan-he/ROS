#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <random>


int main(int argc, char* argv[]){
    ros::init(argc, argv, "randomNumGen");
    ros::NodeHandle handler;
    ros::Publisher pub = handler.advertise<std_msgs::Float32>("/randomNumber", 10);
    std_msgs::Float32 msg;
    float pubRate = 10;
    int maxVal = INT_MAX;
    if (argc != 1){
        pubRate = atof(argv[1]);
        maxVal = atoi(argv[2]);
    }
    ros::Rate rate(pubRate);
    while (ros::ok()){
        msg.data = rand() % maxVal;
        pub.publish(msg);
        rate.sleep();
    }
    return 0;
}