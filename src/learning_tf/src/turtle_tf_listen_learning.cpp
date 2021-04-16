#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

std::string chasingObj;

int main(int argc, char* argv[]){
    ros::init(argc, argv, "liaten_turtle");
    if(argc != 2){
        ROS_ERROR("specify name");
        return -1;
    }
    chasingObj = argv[1];
    ros::NodeHandle handler;
    // 新建二号乌龟，此时turtle2发布的pose被t2broadcast订阅，并发布turtle2相对于world的变换
    ros::service::waitForService("spawn");
    ros::ServiceClient addTurtle = handler.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srvCaller;
    addTurtle.call(srvCaller);
    // 发布控制turtle2速度的通道
    ros::Publisher turtleVel = handler.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    // 要保证tf::TransformListener类一直存在，否则变换缓存会被清空
    tf::TransformListener lsnr;
    ros::Rate rate(10);
    while (ros::ok()){
        tf::StampedTransform stmpTF;
        // try-catch, 防止在二号乌龟生成前无法获取二号乌龟的变换
        try{
            // target& source frameID
            // 根据chasingObj-world和turtle2-world的变换计算turtle2-chasingObj的变换
            lsnr.lookupTransform("turtle2", chasingObj, ros::Time(0), stmpTF);
        }
        catch (tf::TransformException& err){
            ROS_ERROR("%s", err.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        geometry_msgs::Twist velMsg;
        // 控制指令发送到tuetle2的速度控制中，transform节点只负责订阅与发布变换
        velMsg.angular.z = 4.0 * atan2(stmpTF.getOrigin().y(), stmpTF.getOrigin().x());
        velMsg.linear.x = 0.5 * sqrt(pow(stmpTF.getOrigin().x(), 2) + pow(stmpTF.getOrigin().y(), 2));
        turtleVel.publish(velMsg);
        rate.sleep();
    }
    return 0;
}