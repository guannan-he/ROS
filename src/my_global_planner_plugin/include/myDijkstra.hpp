/**
 * @file myDijkstra.hpp
 * @brief dijkstra algorithm for move_base pkg
 * @author guannan-he (guannan-he@outlook.com)
 * @version 1.0
 * @date 2021-05-15
 * 
 * @copyright Copyright (c) {2021}  合肥工业大学-LASIS-何冠男
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-05-15 <td>1.0     <td>guannan-he     <td>内容
 * </table>
 */
#ifndef _MY_DIJKSTRA_
#define _MY_DIJKSTRA_

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

    class myDijkstra : public nav_core::BaseGlobalPlanner{
        public:
        myDijkstra();

        myDijkstra(std::string name, costmap_2d::Costmap2DROS* costmapRos);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        
        private:
        ros::NodeHandle handler_;
        costmap_2d::Costmap2DROS* costmapRos_ = NULL;
        costmap_2d::Costmap2D* costmap_ = NULL;
        bool initialized_ = false;
        int width = -1, height = -1, mapSize = -1;
        /**
         * @brief 有效栅格
         */
        std::vector<bool> OCM;
        /**
         * @brief Get the Index From Map
         * @param  pose             geometry_msgs::PoseStamped
         * @return int 
         */
        int getIndexFromMap(const geometry_msgs::PoseStamped& pose);
        /**
         * @brief 
         * @param  x                cell x
         * @param  y                cell y
         * @return true 
         * @return false 
         */
        bool inBoundary(int x, int y);
        /**
         * @brief Get the Neighbour Index around the cell, 8 directions
         * @param  cellIndex        My Param doc
         * @return std::vector<int> 
         */
        std::vector<int> getNeighbourIndex(int cellIndex);
        /**
         * @brief Get the Move Cost from one cell to another cell
         * @param  firstIndex       My Param doc
         * @param  secondIndex      My Param doc
         * @return double 
         */
        double getMoveCost(int firstIndex, int secondIndex);
        /**
         * @brief Get the Heuristic from one cell to another cell
         * @param  cell_index       My Param doc
         * @param  goal_index       My Param doc
         * @return double 
         */
        double getHeuristic(int cell_index, int goal_index);
    };
};

namespace global_planner{
    myDijkstra::myDijkstra(){
        return;
    }

    myDijkstra::myDijkstra(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        initialize(name, costmapRos);
        return;
    }

    void myDijkstra::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos){
        if (initialized_){
            ROS_WARN("Dijkstra planner already initialized.");
            return;
        }
        costmapRos_ = costmapRos;
        costmap_ = costmapRos->getCostmap();
        handler_ = ros::NodeHandle("~/" + name);
        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        mapSize = width * height;
        OCM.resize(mapSize);
        for (int i = 0; i < width; i++){
            for(int j = 0; j < height; j++){
                OCM[costmap_->getIndex(i, j)] = costmap_->getCost(i, j) == 0 ? true : false;
            }
        }
        initialized_ = true;
    }

    bool myDijkstra::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
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

        int start_index = getIndexFromMap(start);
        int goal_index = getIndexFromMap(goal);

        std::vector<double> gCost(mapSize, infVal);
        std::vector<int> parent(mapSize, -1);
        std::multiset<dijkstraNode> que;
        gCost[start_index] = 0;
        dijkstraNode current;
        current.cost = 0;
        current.index = start_index;
        que.insert(current);
        while (!que.empty()){
            current = *que.begin();
            que.erase(que.begin());
            if (current.index == goal_index){
                ROS_INFO("found path.");
                break;
            }
            std::vector<int> neighbourIndex = getNeighbourIndex(current.index);
            for (int i = neighbourIndex.size() - 1; i > -1; i--){
                if (parent[neighbourIndex[i]] != -1){
                    //在这跳过的原因是因为搜索范围过大，不寻找最优的了
                    continue;
                }
                gCost[neighbourIndex[i]] = gCost[current.index] + getMoveCost(current.index, neighbourIndex[i]);
                parent[neighbourIndex[i]] = current.index;
                dijkstraNode neighbourNode;
                neighbourNode.index = neighbourIndex[i];
                neighbourNode.cost = gCost[neighbourIndex[i]];
                que.insert(neighbourNode);
            }
        }
        if (parent[goal_index] == -1 || start_index == goal_index){
            ROS_WARN("no feasible plan.");
            return false;
        }
        std::vector<int> res;
        current.index = goal_index;
        while (current.index != start_index){
            res.push_back(current.index);
            current.index = parent[current.index];
        }
        res.push_back(start_index);
        ros::Time planTime = ros::Time::now();
        tf::Quaternion zeroOri;
        zeroOri.setRPY(0, 0, 0);
        for (int i = res.size() - 1; i > -1; i--){
            unsigned int tmpX, tmpY;
            costmap_->indexToCells(res[i], tmpX, tmpY);
            double wx, wy;
            costmap_->mapToWorld(tmpX, tmpY, wx, wy);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = costmapRos_->getGlobalFrameID();
            pose.header.stamp = planTime;
            tf::quaternionTFToMsg(zeroOri, pose.pose.orientation);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            plan.push_back(pose);
        }
        return true;
    }

    int myDijkstra::getIndexFromMap(const geometry_msgs::PoseStamped& pose){//从pose获取index
        double wx = pose.pose.position.x;
        double wy = pose.pose.position.y;
        unsigned int mx, my;
        costmap_->worldToMap(wx, wy, mx, my);
        return costmap_->getIndex(mx, my);
    }

    bool myDijkstra::inBoundary(int x, int y){//确保x, y在边界线内
        return x > -1 && y > -1 && x < width && y < height;
    }

    std::vector<int> myDijkstra::getNeighbourIndex(int cellIndex){//获取相邻编号
        unsigned int mx, my;
        costmap_->indexToCells(cellIndex, mx, my);
        std::vector<int> res;
        for (int i = -1; i <= 1; i++){
            for (int j = -1; j <= 1; j++){
                int tmpx = mx + i, tmpy = my + j;
                int tmp = costmap_->getIndex(tmpx, tmpy);
                if (!( i == 0 && j == 0) && inBoundary(tmpx, tmpy) && OCM[tmp]){
                    res.push_back(costmap_->getIndex(tmpx, tmpy));
                }
            }
        }
        return res;
    }

    double myDijkstra::getMoveCost(int firstIndex, int secondIndex){//从邻居移动到当前点的几何距离
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;
        
        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Dijkstra global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1){
            return 1.0;
        }
        return 1.4;
    }
};


#endif