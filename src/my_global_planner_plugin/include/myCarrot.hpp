#ifndef _MY_CAARROT_
#define _MY_CAARROT_

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

// 接口

namespace global_planner{
    // 从nav_core::BaseGlobalPlanner继承接口
    class myCarrot : public nav_core::BaseGlobalPlanner{
        public:
        myCarrot();

        myCarrot(std::string name, costmap_2d::Costmap2DROS* costmapRos);

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

// 实现

namespace global_planner{
    myCarrot::myCarrot(){
        return;
    };

    myCarrot::myCarrot(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        initialize(name, costmapRos);
        return;
    };

    void myCarrot::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        if (initialized_){
            ROS_WARN("carrot planner already initialized.");
            return;
        }
        costmapRos_ = costmapRos;
        costmap_ = costmapRos->getCostmap();
        handler_ = ros::NodeHandle("~/" + name);
        handler_.param("step_size", stepSize_, costmap_->getResolution());
        handler_.param("min_dist_from_robot", minDist_, 0.1);
        worldModel_ = new base_local_planner::CostmapModel(*costmap_);
        initialized_ = true;
        return;
    }

    double myCarrot::footprintCost(double x, double y, double theta){
        //判断该点是不是有效点
        if (!initialized_){
            ROS_WARN("carrot not initialized yet.");
            return -1.0;
        }
        std::vector<geometry_msgs::Point> footprint(costmapRos_->getRobotFootprint());
        if (footprint.size() < 3){
            ROS_WARN("footprint.size() < 3.");
            return -1.0;
        }
        return worldModel_->footprintCost(x, y, theta, footprint);
    }

    bool myCarrot::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        // 就是做一条射线，然后遇到障碍就停
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
        
        plan.clear();
        costmap_ = costmapRos_->getCostmap();
        tf::Stamped<tf::Pose> startTF, goalTF;
        tf::poseStampedMsgToTF(start, startTF);
        tf::poseStampedMsgToTF(goal, goalTF);

        double startYaw, goalYaw, uselessPitch, uselessRoll;
        startTF.getBasis().getEulerYPR(startYaw, uselessPitch, uselessRoll);
        goalTF.getBasis().getEulerYPR(goalYaw, uselessPitch, uselessRoll);

        bool done = false;
        double scale = 0;
        double dScale = 0.01;
        double goalX = goal.pose.position.x;
        double goalY = goal.pose.position.y;
        double startX = start.pose.position.x;
        double startY = start.pose.position.y;
        double diffX = (goalX - startX) * dScale;
        double diffY = (goalY - startY) * dScale;
        double diffYaw = angles::normalize_angle(goalYaw-startYaw);
        double targetX = startX;
        double targetY = startY;
        double targetYaw = startYaw;

        while (!done && scale <= 1.0){
            targetX += diffX;
            targetY += diffY;
            targetYaw = angles::normalize_angle(targetYaw + diffYaw);
            if (footprintCost(targetX, targetY, targetYaw) < 0){
                if (scale < dScale){
                    ROS_WARN("no feasible plan.");
                    return false;
                }
                targetX -= diffX;
                targetY -= diffY;
                targetYaw = angles::normalize_angle(targetYaw - diffYaw);
                done = true;
            }
            scale += dScale;
        }
        plan.push_back(start);
        geometry_msgs::PoseStamped newGoal = goal;
        tf::Quaternion goalQuat = tf::createQuaternionFromYaw(targetYaw);
        newGoal.pose.position.x = targetX;
        newGoal.pose.position.y = targetY;
        tf::quaternionTFToMsg(goalQuat, newGoal.pose.orientation);
        plan.push_back(newGoal);
        return done;
    }
};

#endif