#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class pathController{
    public:
    pathController(){
        return;
    }
    private:
    ros::NodeHandle handler_;
    ros::Subscriber odomSub_;
    ros::Subscriber pathSub_;
    ros::Publisher ackermannPub_;
    geometry_msgs::Twist twist_;
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "path_pure_pursuit_node");
    ros::spin();
    return 0;
}