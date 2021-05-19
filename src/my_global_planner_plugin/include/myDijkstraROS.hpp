#ifndef _MY_DIJKSTRA_ROS_
#define _MY_DIJKSTRA_ROS_

#include <myDijkstraKernel.hpp>
#include <my_global_planner_common.hpp>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <vector>
#include <string>
#include <queue>
#include <set>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace global_planner{
    class myRosDijkstra : public nav_core::BaseGlobalPlanner{
        public:
        myRosDijkstra();
        myRosDijkstra(std::string name, costmap_2d::Costmap2DROS* costmapRos);
        ~myRosDijkstra();
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        private:
        bool allowUnknow_ = false;
        global_planner::dijkstraKernel* kernel_;
        ros::NodeHandle handler_;
        costmap_2d::Costmap2DROS* costmapRos_ = NULL;
        costmap_2d::Costmap2D* costmap_ = NULL;
        bool initialized_ = false;
        int width = -1, height = -1, mapSize = -1;
        float resolution_;
        float originX_, originY_;
        void processPath(std::vector<geometry_msgs::PoseStamped>& plan);
    };
};

namespace global_planner{
    myRosDijkstra::myRosDijkstra(){
        return;
    }
    myRosDijkstra::myRosDijkstra(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        initialize(name, costmapRos);
        return;
    }
    myRosDijkstra::~myRosDijkstra(){
        delete kernel_;
    }
    void myRosDijkstra::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        if (initialized_){
            ROS_WARN("Dijkstra planner already initialized.");
            return;
        }
        costmapRos_ = costmapRos;
        costmap_ = costmapRos->getCostmap();
        handler_ = ros::NodeHandle("~/" + name);
        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution_ = costmap_->getResolution();
        originX_ = costmap_->getOriginX();
        originY_ = costmap_->getOriginY();
        handler_.param("allow_unknown", initialized_, true);

        kernel_ = new dijkstraKernel(width, height);
        kernel_->setCostmap(costmap_->getCharMap(), true, allowUnknow_);
        initialized_ = true;
        return;
    }
    bool myRosDijkstra::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
         if (!initialized_){
            ROS_WARN("carrot not initialized yet.");
            return false;
        }
        ROS_DEBUG("plan request received.");
        if (goal.header.frame_id != costmapRos_->getGlobalFrameID()){
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", costmapRos_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }
        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
        int x = (int)round((start.pose.position.x - originX_) / resolution_);
        int y = (int)round((start.pose.position.y - originY_) / resolution_);
        kernel_->setStart(x, y);
        x = (int)round((goal.pose.position.x - originX_) / resolution_);
        y = (int)round((goal.pose.position.y - originY_) / resolution_);
        kernel_->setGoal(x, y);
        if (!kernel_->dijkstraPath()){
            return false;
        }
        processPath(plan);
        return true;
    }
    void myRosDijkstra::processPath(std::vector<geometry_msgs::PoseStamped>& plan){
        float* pathX = kernel_->getPathX();
        float* pathY = kernel_->getPathY();
        int pathLen = kernel_->getPathLen();
        std::string frameID = "map";
        ros::Time nowTime = ros::Time::now();
        // ROS_INFO("%d", pathLen);
        plan.resize(pathLen);
        tf::Quaternion zeroRot_;
        zeroRot_.setRPY(0, 0, 0);
        for (int i = 0; i < pathLen; i++){
            geometry_msgs::PoseStamped& pose = plan[i];
            pose.header.frame_id = frameID;
            pose.header.stamp = nowTime;
            pose.pose.position.x = pathX[i] * resolution_ + originX_;
            pose.pose.position.y = pathY[i] * resolution_ + originY_;
            pose.pose.position.z = 0;
            tf::quaternionTFToMsg(zeroRot_, pose.pose.orientation);
        }
        return;
    }
};

#endif