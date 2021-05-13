#ifndef _MYDIJKSTRA_
#define _MYDIJKSTRA_

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
        double footprintCost(double x, double y, double theta);
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
        initialized_ = true;
        return;
    }
};


#endif