#ifndef _MY_DIJKSTRA_
#define _MY_DIJKSTRA_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>

namespace global_planner{
    class myDijkstra: public nav_core::BaseGlobalPlanner{
        public:
        myDijkstra();
        myDijkstra(std::string name, costmap_2d::Costmap2DROS* costmapRos);
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        private:
        ros::NodeHandle handler_;
        costmap_2d::Costmap2DROS* costmapRos_ = NULL;
        costmap_2d::Costmap2D* costmap_ = NULL;
        base_local_planner::WorldModel* worldModel_ = NULL;
        double stepSize_ = -1.0, minDist_ = -1.0;
        bool initialized_ = false;
        std::vector<geometry_msgs::Point> footprint_;
        double footprintCost(double x, double y, double theta);
        double footprintCost(const geometry_msgs::PoseStamped& start);
    };
};

namespace global_planner{
    myDijkstra::myDijkstra(){
        return;
    }
    myDijkstra::myDijkstra(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        initialize(name, costmapRos);
    }
    void myDijkstra::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        if(initialized_){
            ROS_WARN("dijkstra planner already initialized.");
            return;
        }
        costmapRos_ = costmapRos;
        costmap_ = costmapRos_->getCostmap();
        handler_ = ros::NodeHandle("~/" + name);
        handler_.param("step_size", stepSize_, costmap_->getResolution());
        handler_.param("min_dist_from_robot", minDist_, 0.1);
        worldModel_ = new base_local_planner::CostmapModel(*costmap_);
        footprint_ = costmapRos_->getRobotFootprint();
        if (footprint_.size() < 3){
            ROS_WARN("footprint.size() < 3.");
            return;
        }
        initialized_ = true;
        return;
    }
    double myDijkstra::footprintCost(double x, double y, double theta){
        //判断该点是不是有效点
        if (!initialized_){
            ROS_WARN("dikjstra not initialized yet.");
            return -1.0;
        }
        return worldModel_->footprintCost(x, y, theta, footprint_);
    }
    double myDijkstra::footprintCost(const geometry_msgs::PoseStamped& point){
        tf::Stamped<tf::Pose> pointPose;
        tf::poseStampedMsgToTF(point, pointPose);
        double uselessRoll, uselessPitch, yaw;
        pointPose.getBasis().getEulerYPR(yaw, uselessPitch, uselessRoll);
        return footprintCost(point.pose.position.x, point.pose.position.y, yaw);
    }
    bool myDijkstra::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        if (!initialized_){
            ROS_WARN("dikjstra not initialized yet.");
            return false;
        }
        ROS_INFO("plan request received.");
        if (footprintCost(start) < 0){
            ROS_WARN("invalid start.");
            return false;
        }
        if (footprintCost(goal) < 0){
            ROS_WARN("invalid goal.");
            return false;
        }
        if (goal.header.frame_id != costmapRos_->getGlobalFrameID()){
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", costmapRos_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }
        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
        plan.clear();

        // to be achieved. 20210513

    }
};


#endif