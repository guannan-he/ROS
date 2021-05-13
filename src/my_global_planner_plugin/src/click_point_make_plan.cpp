#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base/move_base.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class planCaller{
    public:
    planCaller(){
        pathPub_ = handler_.advertise<nav_msgs::Path>("my_global_planned_path", 10);
        planClient_ = handler_.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
        ROS_INFO("click_point_make_plan: waiting for service.");
        planClient_.waitForExistence();
        ROS_INFO("click_point_make_plan: service established.");
        goalSub_ = handler_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &planCaller::callback, this);
        return;
    }
    private:
    ros::NodeHandle handler_;
    ros::Publisher pathPub_;
    ros::Subscriber goalSub_;
    nav_msgs::GetPlan request_;
    ros::ServiceClient planClient_;
    tf::TransformListener tfListener_;
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg){
        ROS_INFO("click_point_make_plan: point received.");
        tf::StampedTransform stmpTF;
        tfListener_.lookupTransform("map", "base_link", ros::Time(0), stmpTF);
        tf::Quaternion zeroRot;
        zeroRot.setRPY(0, 0, 0);
        tf::pointTFToMsg(stmpTF.getOrigin(), request_.request.start.pose.position);
        tf::quaternionTFToMsg(stmpTF.getRotation(), request_.request.start.pose.orientation);
        request_.request.goal.pose.position = msg->point;
        request_.request.goal.header = msg->header;
        request_.request.start.header = msg->header;
        tf::quaternionTFToMsg(zeroRot, request_.request.goal.pose.orientation);
        planClient_.call(request_);
        request_.response.plan.header = msg->header;
        pathPub_.publish(request_.response.plan);
        ROS_INFO("click_point_make_plan: plan published.");
        return;
    }
};


int main(int argc, char* argv[]){
    ros::init(argc, argv, "click_point_make_plan");
    planCaller caller;
    ros::spin();
    return 0;
}