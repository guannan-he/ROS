#include <ros/ros.h>
#include <my_global_planner_common.hpp>
#include <myDijkstraKernel.hpp>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define KERNEL_TYPE global_planner::dijkstraKernel

//*(char(*)[100])map_

class planCaller{
    public:
    planCaller(){
        pathPub_ = handler_.advertise<nav_msgs::Path>("my_global_planned_path", 10);
        goalSub_ = handler_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &planCaller::callback, this);
        mapClient_ = handler_.serviceClient<nav_msgs::GetMap>("/static_map");
        ROS_INFO("kernelDebug: waiting for service.");
        mapClient_.waitForExistence();
        ROS_INFO("kernelDebug: map service established.");
        mapClient_.call(mapRequest_);
        mapHeight_ = mapRequest_.response.map.info.height;
        mapWidth_ = mapRequest_.response.map.info.width;
        resolution_ = mapRequest_.response.map.info.resolution;
        originX_ = mapRequest_.response.map.info.origin.position.x;
        originY_ = mapRequest_.response.map.info.origin.position.y;

        map_ = new KERNEL_COSTMAP_TYPE[mapHeight_ * mapWidth_];
        memcpy(map_, &mapRequest_.response.map.data[0], mapHeight_ * mapWidth_ * sizeof(KERNEL_COSTMAP_TYPE));
        kernel_ = new KERNEL_TYPE(mapWidth_, mapHeight_);
        kernel_->setCostmap(map_, false);

        zeroRot_.setRPY(0, 0, 0);
        pathMsg_.header.seq = 0;
        return;
    }
    private:
    ros::NodeHandle handler_;
    ros::Publisher pathPub_;
    ros::Subscriber goalSub_;
    ros::ServiceClient mapClient_;
    nav_msgs::GetMap mapRequest_;
    tf::Quaternion zeroRot_;
    nav_msgs::Path pathMsg_;
    KERNEL_TYPE* kernel_;
    KERNEL_COSTMAP_TYPE* map_;
    int mapHeight_, mapWidth_;
    float resolution_;
    float originX_, originY_;
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg){
        int x = (int)round((msg->point.x - originX_) / resolution_);
        int y = (int)round((msg->point.y - originY_) / resolution_);
        if (msg->header.seq % 2 == 0){
            ROS_INFO("kernelDebug: start point received.");
            kernel_->setStart(x, y);
        }
        else{
            ROS_INFO("kernelDebug: goal point received.");
            kernel_->setGoal(x, y);
            if (kernel_->dijkstraPath()){
                processPath();
                pathPub_.publish(pathMsg_);
                ROS_INFO("kernelDebug: plan published.");
            }
        }
        return;
    }
    void processPath(){
        float* pathX = kernel_->getPathX();
        float* pathY = kernel_->getPathY();
        int pathLen = kernel_->getPathLen();
        std::string frameID = "map";
        ros::Time nowTime = ros::Time::now();
        int seq = pathMsg_.header.seq + 1;

        pathMsg_.header.frame_id = frameID;
        pathMsg_.header.seq = seq;
        pathMsg_.header.stamp = nowTime;
        pathMsg_.poses.resize(pathLen);

        for (int i = 0; i < pathLen; i++){
            geometry_msgs::PoseStamped& pose = pathMsg_.poses[i];
            pose.header.frame_id = frameID;
            pose.header.seq = seq;
            pose.header.stamp = nowTime;
            pose.pose.position.x = pathX[i] * resolution_ + originX_;
            pose.pose.position.y = pathY[i] * resolution_ + originY_;
            pose.pose.position.z = 0;
            tf::quaternionTFToMsg(zeroRot_, pose.pose.orientation);
        }
        return;
    }
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "kernelDebug");
    planCaller caller;
    ros::spin();
    return 0;
}