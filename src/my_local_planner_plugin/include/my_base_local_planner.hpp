/**
 * @file my_base_local_planner.hpp
 * @brief kernel of base_local_planner, copied from base_local_planner/base_local_planner.h and ~.cpp
 * @author guannan-he (guannan-he@outlook.com)
 * @version 1.0
 * @date 2021-06-09
 * 
 * @copyright Copyright (c) {2021}  合肥工业大学-LASIS-何冠男
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-06-09 <td>1.0     <td>guannan-he     <td>内容
 * </table>
 */
#ifndef _MY_BASE_LOCAL_PLANNER_
#define _MY_BASE_LOCAL_PLANNER_

// #define PANNER_DEBUG_MODE

#include <vector>
#include <cmath>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <base_local_planner/footprint_helper.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>

//we'll take in a path as a vector of poses
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

//for creating a local cost grid
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>

#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>



#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>

/**
 * @brief 类定义
 */
namespace base_local_planner{
    class my_local_planner_kernel{
        public:
        /**
         * @brief Construct a new my local planner kernel object
         * @param  world_model                     检测轮廓是否碰到障碍
         * @param  costmap                         局部代价图
         * @param  footprint_spec                  轮廓
         * @param  acc_lim_x                       acc_lim_x
         * @param  acc_lim_y                       acc_lim_y
         * @param  acc_lim_theta                   acc_lim_theta
         * @param  sim_time                        仿真时间
         * @param  sim_granularity                 仿真位移分辨率
         * @param  vx_samples                      纵向速度采样数
         * @param  vtheta_samples                  横摆角速度采样数
         * @param  path_distance_bias              路径距离权重
         * @param  goal_distance_bias              终点距离权重
         * @param  occdist_scale                   障碍权重
         * @param  heading_lookahead               朝向角预眇距离
         * @param  oscillation_reset_dist          震荡重置距离
         * @param  escape_reset_dist               逃逸重置距离
         * @param  escape_reset_theta              逃逸重置角度
         * @param  holonomic_robot                 全约束机器人
         * @param  max_vel_x                       max_vel_x
         * @param  min_vel_x                       min_vel_x
         * @param  max_vel_th                      max_vel_th
         * @param  min_vel_th                      min_vel_th
         * @param  min_in_place_vel_th             min_vel_th 原地
         * @param  backup_vel                      倒车速度
         * @param  dwa                             动态窗口法
         * @param  heading_scoring                 航向角是否考虑
         * @param  heading_scoring_timestep        某一时刻取航向角
         * @param  meter_scoring                   选择cell代价还是world代价
         * @param  simple_attractor                只用终点作追踪点
         * @param  y_vels                          可用横向速度表
         * @param  stop_time_buffer                停车TTC
         * @param  sim_period                      DWA 仿真时间
         * @param  angular_sim_granularity         仿真角速度分辨率
         */
        my_local_planner_kernel(
            WorldModel& world_model, 
            const costmap_2d::Costmap2D& costmap, 
            std::vector<geometry_msgs::Point> footprint_spec,
            double acc_lim_x = 1.0, double acc_lim_y = 1.0, double acc_lim_theta = 1.0,
            double sim_time = 1.0, double sim_granularity = 0.025, 
            int vx_samples = 20, int vtheta_samples = 20,
            double path_distance_bias = 0.6, double goal_distance_bias = 0.8, double occdist_scale = 0.2,
            double heading_lookahead = 0.325, double oscillation_reset_dist = 0.05, 
            double escape_reset_dist = 0.10, double escape_reset_theta = M_PI_2,
            bool holonomic_robot = true,
            double max_vel_x = 0.5, double min_vel_x = 0.1, 
            double max_vel_th = 1.0, double min_vel_th = -1.0, double min_in_place_vel_th = 0.4,
            double backup_vel = -0.1,
            bool dwa = false, bool heading_scoring = false, double heading_scoring_timestep = 0.1,
            bool meter_scoring = true,
            bool simple_attractor = false,
            std::vector<double> y_vels = std::vector<double>(0),
            double stop_time_buffer = 0.2,
            double sim_period = 0.1, double angular_sim_granularity = 0.025
        );

        /**
         * @brief Destroy the my local planner kernel object
         */
        ~my_local_planner_kernel();

        /**
         * @brief 参数动态调节回调
         * @param  cfg                             callback msg
         */
        void reconfigure(BaseLocalPlannerConfig &cfg);

        /**
         * @brief 搜索最佳轨迹, ROS wrapper 调用
         * @param  global_pose                     在世界中的位姿
         * @param  global_vel                      世界中的速度
         * @param  drive_velocities                底盘速度控制命令
         * @return 最佳轨迹 
         */
        Trajectory findBestPath(const geometry_msgs::PoseStamped& global_pose, geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities);
        
        /**
         * @brief 更新局部的全局路径, ROS wrapper 调用
         * @param  new_plan                        路径点
         * @param  compute_dists                   更新时计算距离图
         */
        void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists = false);
        
        /**
         * @brief 获取当前终点
         * @param  x                               &x 世界坐标
         * @param  y                               &y 世界坐标
         */
        void getLocalGoal(double& x, double& y);

        /**
         * @brief 生成一条轨迹并检查是否合法
         * @param  x                               x pos
         * @param  y                               y pos
         * @param  theta                           yaw
         * @param  vx                              x vel
         * @param  vy                              y vel
         * @param  vtheta                          yaw rate
         * @param  vx_samp                         desired x vel
         * @param  vy_samp                         desired y vel
         * @param  vtheta_samp                     desired yaw rate
         * @return 合法轨迹 
         */
        bool checkTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

        /**
         * @brief 生成一条轨迹并检查是否合法
         * @param  pos                             geometry_msgs::Pose pos
         * @param  vel                             geometry_msgs::Twist vel
         * @param  vx_samp                         vx_samp
         * @param  vy_samp                         vy_samp
         * @param  vtheta_samp                     vtheta_samp
         * @return true 
         * @return false 
         */
        bool checkTrajectory(geometry_msgs::Pose pos, geometry_msgs::Twist vel, double vx_samp, double vy_samp, double vtheta_samp);

        /**
         * @brief 生成一条轨迹并检查是否合法
         * @param  pos                             geometry_msgs::Pose pos
         * @param  vel                             geometry_msgs::Pose vel
         * @param  vx_samp                         vx_samp
         * @param  vy_samp                         vy_samp
         * @param  vtheta_samp                     vtheta_samp
         * @return true 
         * @return false 
         */
        bool checkTrajectory(geometry_msgs::Pose pos, geometry_msgs::Pose vel, double vx_samp, double vy_samp, double vtheta_samp);

        /**
         * @brief 生成一条轨迹并返回代价
         * @param  x                               x pos
         * @param  y                               y pos
         * @param  theta                           yaw
         * @param  vx                              x vel
         * @param  vy                              y vel
         * @param  vtheta                          yaw rate
         * @param  vx_samp                         desired x vel
         * @param  vy_samp                         desired y vel
         * @param  vtheta_samp                     desired yaw rate
         * @return 轨迹代价 
         */
        double scoreTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

        /**
         * @brief 生成一条轨迹并返回代价
         * @param  pos                             geometry_msgs::Pose pos
         * @param  vel                             geometry_msgs::Twist vel
         * @param  vx_samp                         vx_samp
         * @param  vy_samp                         vy_samp
         * @param  vtheta_samp                     vtheta_samp
         * @return true 
         * @return false 
         */
        bool scoreTrajectory(geometry_msgs::Pose pos, geometry_msgs::Twist vel, double vx_samp, double vy_samp, double vtheta_samp);

        /**
         * @brief 生成一条轨迹并检查是否合法
         * @param  pos                             geometry_msgs::Pose pos
         * @param  vel                             geometry_msgs::Pose vel
         * @param  vx_samp                         vx_samp
         * @param  vy_samp                         vy_samp
         * @param  vtheta_samp                     vtheta_samp
         * @return true 
         * @return false 
         */
        bool scoreTrajectory(geometry_msgs::Pose pos, geometry_msgs::Pose vel, double vx_samp, double vy_samp, double vtheta_samp);
        
        /**
         * @brief Get the Cell Costs
         * @param  cx                              x cell
         * @param  cy                              y cell
         * @param  path_cost                       &path_cost
         * @param  goal_cost                       &goal_cost
         * @param  occ_cost                        &occ_cost
         * @param  total_cost                      &total_cost
         * @return 单元格可移动
         */
        bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

        /**
         * @brief Set the Footprint object
         * @param  footprint                       轮廓
         */
        void setFootprint(std::vector<geometry_msgs::Point> footprint);

        /**
         * @brief Get the Footprint Polygon object
         * @return geometry_msgs::Polygon 
         */
        geometry_msgs::Polygon getFootprintPolygon();

        /**
         * @brief Get the Footprint object
         * @return std::vector<geometry_msgs::Point> 
         */
        std::vector<geometry_msgs::Point> getFootprint();

        private:
        /**
         * @brief Create a Trajectories object, 内部调用
         * @param  x                               x pos
         * @param  y                               y pos
         * @param  theta                           yaw
         * @param  vx                              x vel
         * @param  vy                              y vel
         * @param  vtheta                          yaw rate
         * @param  acc_x                           acc_x
         * @param  acc_y                           acc_y
         * @param  acc_theta                       acc_theta
         * @return 最优轨迹 
         */
        Trajectory createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta, double acc_x, double acc_y, double acc_theta);
        
        /**
         * @brief 生成一条轨迹， 内部调用
         * @param  x                               x pos
         * @param  y                               y pos
         * @param  theta                           yaw
         * @param  vx                              x vel
         * @param  vy                              y vel
         * @param  vtheta                          yaw rate
         * @param  vx_samp                         desired x vel
         * @param  vy_samp                         desired y vel
         * @param  vtheta_samp                     desired yaw rate
         * @param  acc_x                           acc_x
         * @param  acc_y                           acc_y
         * @param  acc_theta                       acc_theta
         * @param  impossible_cost                 障碍代价
         * @param  traj                            &Trajectory
         */
        void generateTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y, double acc_theta, double impossible_cost, Trajectory& traj);
        
        /**
         * @brief 获取某位姿轮廓代价
         * @param  x_i                             x pos
         * @param  y_i                             y pos
         * @param  theta_i                         yaw
         * @return double 
         */
        double footprintCost(double x_i, double y_i, double theta_i);

        /**
         * @brief 下一时刻x位置
         * @param  xi                              x pos
         * @param  vx                              x vel
         * @param  vy                              y vel
         * @param  theta                           yaw rate
         * @param  dt                              dt
         * @return next x pos 
         */
        inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt);

        /**
         * @brief 下一时刻y位置
         * @param  yi                              y pos
         * @param  vx                              x vel
         * @param  vy                              y vel
         * @param  theta                           yaw rate
         * @param  dt                              dt
         * @return next y pos 
         */
        inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt);

        /**
         * @brief 下一时刻yaw
         * @param  thetai                          yaw
         * @param  vth                             yaw rate
         * @param  dt                              dt
         * @return next yaw 
         */
        inline double computeNewThetaPosition(double thetai, double vth, double dt);

        /**
         * @brief 下一时刻速度
         * @param  vg                              goal
         * @param  vi                              current
         * @param  a_max                           acc lim
         * @param  dt                              dt
         * @return next vel 
         */
        inline double computeNewVelocity(double vg, double vi, double a_max, double dt);

        /**
         * @brief TTC速度限制-最大减速度
         * @param  time                            TTC
         * @param  vx                              &vx
         * @param  vy                              &vy
         * @param  vth                             &yaw rate
         */
        void getMaxSpeedToStopInTime(double time, double& vx, double& vy, double& vth);
        
        /**
         * @brief 布雷森汉姆直线算法--求线上最大代价
         * @param  x0                              x0
         * @param  x1                              x1
         * @param  y0                              y0
         * @param  y1                              y1
         * @return 线上最大代价 
         */
        double lineCost(int x0, int x1, int y0, int y1);

        /**
         * @brief 求点代价
         * @param  x                               x
         * @param  y                               y
         * @return 点代价 
         */
        double pointCost(int x, int y);

        /**
         * @brief 当前点到轨迹终点夹角与当前航向角差
         * @param  cell_x                          cell_x
         * @param  cell_y                          cell_y
         * @param  x                               x pos
         * @param  y                               y pos
         * @param  heading                         yaw
         * @return 航向角差 
         */
        double headingDiff(int cell_x, int cell_y, double x, double y, double heading);

        base_local_planner::FootprintHelper footprint_helper_;
        MapGrid path_map_; ///< @brief 各个点到路径点最近距离
        MapGrid goal_map_; ///< @brief 各个点到路径上最后一个点距离
        const costmap_2d::Costmap2D& costmap_; ///< @brief 局部代价图引用
        WorldModel& world_model_; ///< @brief 检测轮廓是否碰到障碍
        std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief 底盘轮廓点
        std::vector<geometry_msgs::PoseStamped> global_plan_; ///< @brief 裁减过，在局部路代价图中的全局路线（第一部分）

        bool stuck_left_, stuck_right_; ///< @brief 防止原地转向震荡
        bool rotating_left_, rotating_right_; ///< @brief 原地转向方向

        bool stuck_left_strafe_, stuck_right_strafe_; ///< @brief Booleans to keep the robot from oscillating during strafing
        bool strafe_right_, strafe_left_; ///< @brief Booleans to keep track of strafe direction for the robot

        bool escaping_; ///< @brief 逃逸模式
        bool meter_scoring_;///< @brief 选择cell代价还是world代价

        double goal_x_,goal_y_; ///< @brief Storage for the local goal the robot is pursuing

        double final_goal_x_, final_goal_y_; ///< @brief 当前终点，作为追踪点
        bool final_goal_position_valid_; ///< @brief 终点是否有效

        double sim_time_; ///< @brief 轨迹展开仿真时间
        double sim_granularity_; ///< @brief 仿真位移分辨率
        double angular_sim_granularity_; ///< @brief 仿真角速度分辨率

        //控制空间内采样
        int vx_samples_; ///< @brief 纵向速度采样数量
        int vtheta_samples_; ///< @brief 横摆教速度采样

        double path_distance_bias_, goal_distance_bias_, occdist_scale_; ///< @brief 代价系数
        double acc_lim_x_, acc_lim_y_, acc_lim_theta_; ///< @brief 加速度限制

        double prev_x_, prev_y_; ///< @brief 上一个走过的点
        double escape_x_, escape_y_, escape_theta_; ///< @brief 上一个处于逃逸状态的点，用于置零escaping_

        Trajectory traj_one_, traj_two_; ///< @brief 最好和次好轨迹

        double heading_lookahead_; ///< @brief 朝向角预眇距离
        double oscillation_reset_dist_; ///< @brief 震荡重置距离
        double escape_reset_dist_, escape_reset_theta_; ///< @brief 逃逸重置距离与角度
        bool holonomic_robot_; ///< @brief 全约束机器人 

        double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_; ///< @brief 速度限制

        double backup_vel_; ///< @brief 倒车速度

        bool dwa_;  ///< @brief 动态窗口法
        bool heading_scoring_; ///< @brief 航向角是否考虑
        double heading_scoring_timestep_; ///< @brief 某一时刻取航向角
        bool simple_attractor_;  ///< @brief 只用终点作追踪点

        std::vector<double> y_vels_; ///< @brief Y 可用横向速度表

        double stop_time_buffer_; ///< @brief 停车TTC
        double sim_period_; ///< @brief DWA 计算速度时间

        double inscribed_radius_, circumscribed_radius_;  ///< @brief 轮廓相关

        boost::mutex configuration_mutex_;  ///< @brief 线程互斥锁
    };
};


/**
 * @brief 辅助函数
 */
namespace base_local_planner{
    void my_local_planner_kernel::reconfigure(BaseLocalPlannerConfig &cfg){
        BaseLocalPlannerConfig config(cfg);
        boost::mutex::scoped_lock l(configuration_mutex_);

        acc_lim_x_ = config.acc_lim_x;
        acc_lim_y_ = config.acc_lim_y;
        acc_lim_theta_ = config.acc_lim_theta;

        max_vel_x_ = config.max_vel_x;
        min_vel_x_ = config.min_vel_x;

        max_vel_th_ = config.max_vel_theta;
        min_vel_th_ = config.min_vel_theta;
        min_in_place_vel_th_ = config.min_in_place_vel_theta;

        sim_time_ = config.sim_time;
        sim_granularity_ = config.sim_granularity;
        angular_sim_granularity_ = config.angular_sim_granularity;

        path_distance_bias_ = config.path_distance_bias;
        goal_distance_bias_ = config.goal_distance_bias;
        occdist_scale_ = config.occdist_scale;

        if (meter_scoring_) {
            //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
            double resolution = costmap_.getResolution();
            goal_distance_bias_ *= resolution;
            path_distance_bias_ *= resolution;
        }

        oscillation_reset_dist_ = config.oscillation_reset_dist;
        escape_reset_dist_ = config.escape_reset_dist;
        escape_reset_theta_ = config.escape_reset_theta;

        vx_samples_ = config.vx_samples;
        vtheta_samples_ = config.vtheta_samples;

        if (vx_samples_ <= 0) {
            config.vx_samples = 1;
            vx_samples_ = config.vx_samples;
            ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
        }
        if(vtheta_samples_ <= 0) {
            config.vtheta_samples = 1;
            vtheta_samples_ = config.vtheta_samples;
            ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
        }

        heading_lookahead_ = config.heading_lookahead;

        holonomic_robot_ = config.holonomic_robot;
        
        backup_vel_ = config.escape_vel;

        dwa_ = config.dwa;

        heading_scoring_ = config.heading_scoring;
        heading_scoring_timestep_ = config.heading_scoring_timestep;

        simple_attractor_ = config.simple_attractor;

        std::string y_string = config.y_vels;
        std::vector<std::string> y_strs;
        boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

        std::vector<double> y_vels;
        for(std::vector<std::string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
            std::istringstream iss(*it);
            double temp;
            iss >> temp;
            y_vels.push_back(temp);
            //ROS_INFO("Adding y_vel: %e", temp);
        }

        y_vels_ = y_vels;
        return;
    }
    void my_local_planner_kernel::getLocalGoal(double& x, double& y){
        x = path_map_.goal_x_;
        y = path_map_.goal_y_;
    }
    bool my_local_planner_kernel::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
        MapCell cell = path_map_(cx, cy);
        MapCell goal_cell = goal_map_(cx, cy);
        if (cell.within_robot) {
            return false;
        }
        occ_cost = costmap_.getCost(cx, cy);
        if (cell.target_dist == path_map_.obstacleCosts() ||
            cell.target_dist == path_map_.unreachableCellCosts() ||
            occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return false;
        }
        path_cost = cell.target_dist;
        goal_cost = goal_cell.target_dist;
        total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ * goal_cost + occdist_scale_ * occ_cost;
        return true;
    }
    void my_local_planner_kernel::setFootprint(std::vector<geometry_msgs::Point> footprint){
        footprint_spec_ = footprint;
        return;
    }
    geometry_msgs::Polygon my_local_planner_kernel::getFootprintPolygon(){
        return costmap_2d::toPolygon(footprint_spec_);
    }
    std::vector<geometry_msgs::Point> my_local_planner_kernel::getFootprint(){
        return footprint_spec_;
    }
    double my_local_planner_kernel::footprintCost(double x_i, double y_i, double theta_i){
        return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
    }
    inline double my_local_planner_kernel::computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
        return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
    }
    inline double my_local_planner_kernel::computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
    }
    inline double my_local_planner_kernel::computeNewThetaPosition(double thetai, double vth, double dt){
        return thetai + vth * dt;
    }
    inline double my_local_planner_kernel::computeNewVelocity(double vg, double vi, double a_max, double dt){
        if((vg - vi) >= 0) {
            return std::min(vg, vi + a_max * dt);
        }
        return std::max(vg, vi - a_max * dt);
    }
    void my_local_planner_kernel::getMaxSpeedToStopInTime(double time, double& vx, double& vy, double& vth){
        vx = acc_lim_x_ * std::max(time, 0.0);
        vy = acc_lim_y_ * std::max(time, 0.0);
        vth = acc_lim_theta_ * std::max(time, 0.0);
        return;
    }
    double my_local_planner_kernel::lineCost(int x0, int x1, int y0, int y1){
        //Bresenham Ray-Tracing
        int deltax = abs(x1 - x0);        // The difference between the x's
        int deltay = abs(y1 - y0);        // The difference between the y's
        int x = x0;                       // Start x off at the first pixel
        int y = y0;                       // Start y off at the first pixel

        int xinc1, xinc2, yinc1, yinc2;
        int den, num, numadd, numpixels;

        double line_cost = 0.0;
        double point_cost = -1.0;

        if (x1 >= x0) {                 // The x-values are increasing
            xinc1 = 1;
            xinc2 = 1;
        }
        else {                         // The x-values are decreasing
            xinc1 = -1;
            xinc2 = -1;
        }

        if (y1 >= y0) {                // The y-values are increasing
            yinc1 = 1;
            yinc2 = 1;
        }
        else {                         // The y-values are decreasing
            yinc1 = -1;
            yinc2 = -1;
        }

        if (deltax >= deltay) {        // There is at least one x-value for every y-value
            xinc1 = 0;                  // Don't change the x when numerator >= denominator
            yinc2 = 0;                  // Don't change the y for every iteration
            den = deltax;
            num = deltax / 2;
            numadd = deltay;
            numpixels = deltax;         // There are more x-values than y-values
        }
        else {                      // There is at least one y-value for every x-value
            xinc2 = 0;                  // Don't change the x for every iteration
            yinc1 = 0;                  // Don't change the y when numerator >= denominator
            den = deltay;
            num = deltay / 2;
            numadd = deltax;
            numpixels = deltay;         // There are more y-values than x-values
        }

        for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
            point_cost = pointCost(x, y); //Score the current point

            if (point_cost < 0) {
                return -1;
            }

            if (line_cost < point_cost) {
                line_cost = point_cost;
            }

            num += numadd;              // Increase the numerator by the top of the fraction
            if (num >= den) {           // Check if numerator >= denominator
                num -= den;               // Calculate the new numerator value
                x += xinc1;               // Change the x as appropriate
                y += yinc1;               // Change the y as appropriate
            }
            x += xinc2;                 // Change the x as appropriate
            y += yinc2;                 // Change the y as appropriate
        }

        return line_cost;
    }
    double my_local_planner_kernel::pointCost(int x, int y){
        unsigned char cost = costmap_.getCost(x, y);
        //if the cell is in an obstacle the path is invalid
        if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION){
            return -1;
        }
        return cost;
    }
    double my_local_planner_kernel::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
        unsigned int goal_cell_x, goal_cell_y;
        // find a clear line of sight from the robot's cell to a farthest point on the path
        for (int i = global_plan_.size() - 1; i >=0; --i) {
            if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) {
                if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
                    double gx, gy;
                    costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
                    return fabs(angles::shortest_angular_distance(heading, atan2(gy - y, gx - x)));
                }
            }
        }
        return DBL_MAX;
    }
    bool my_local_planner_kernel::checkTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
        double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);
        if(cost >= 0) {
            return true;
        }
        ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);
        //otherwise the check fails
        return false;
    }
    bool my_local_planner_kernel::checkTrajectory(geometry_msgs::Pose pos, geometry_msgs::Twist vel, double vx_samp, double vy_samp, double vtheta_samp){
        return checkTrajectory(pos.position.x, pos.position.y, tf2::getYaw(pos.orientation), vel.linear.x, vel.linear.y, vel.angular.z, vx_samp, vy_samp, vtheta_samp);
    }
    bool my_local_planner_kernel::checkTrajectory(geometry_msgs::Pose pos, geometry_msgs::Pose vel, double vx_samp, double vy_samp, double vtheta_samp){
        return checkTrajectory(pos.position.x, pos.position.y, tf2::getYaw(pos.orientation), vel.position.x, vel.position.y, tf2::getYaw(vel.orientation), vx_samp, vy_samp, vtheta_samp);
    }
    double my_local_planner_kernel::scoreTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
        Trajectory t;
        double impossible_cost = path_map_.obstacleCosts();
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_lim_x_, acc_lim_y_, acc_lim_theta_, impossible_cost, t);
        return double( t.cost_ );
    }
    bool my_local_planner_kernel::scoreTrajectory(geometry_msgs::Pose pos, geometry_msgs::Twist vel, double vx_samp, double vy_samp, double vtheta_samp){
        return scoreTrajectory(pos.position.x, pos.position.y, tf2::getYaw(pos.orientation), vel.linear.x, vel.linear.y, vel.angular.z, vx_samp, vy_samp, vtheta_samp);
    }
    bool my_local_planner_kernel::scoreTrajectory(geometry_msgs::Pose pos, geometry_msgs::Pose vel, double vx_samp, double vy_samp, double vtheta_samp){
        return scoreTrajectory(pos.position.x, pos.position.y, tf2::getYaw(pos.orientation), vel.position.x, vel.position.y, tf2::getYaw(vel.orientation), vx_samp, vy_samp, vtheta_samp);
    }
};


/**
 * @brief 主函数
 */
namespace base_local_planner{
    my_local_planner_kernel::my_local_planner_kernel(
        WorldModel& world_model,
        const costmap_2d::Costmap2D& costmap,
        std::vector<geometry_msgs::Point> footprint_spec,
        double acc_lim_x, double acc_lim_y, double acc_lim_theta,
        double sim_time, double sim_granularity,
        int vx_samples, int vtheta_samples,
        double path_distance_bias, double goal_distance_bias, double occdist_scale,
        double heading_lookahead, double oscillation_reset_dist,
        double escape_reset_dist, double escape_reset_theta,
        bool holonomic_robot,
        double max_vel_x, double min_vel_x,
        double max_vel_th, double min_vel_th, double min_in_place_vel_th,
        double backup_vel,
        bool dwa, bool heading_scoring, double heading_scoring_timestep, bool meter_scoring, bool simple_attractor,
        std::vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity
        ):
        path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
        goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
        costmap_(costmap),
        world_model_(world_model), footprint_spec_(footprint_spec),
        sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
        vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
        path_distance_bias_(path_distance_bias), goal_distance_bias_(goal_distance_bias), occdist_scale_(occdist_scale),
        acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
        prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead),
        oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
        escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
        max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
        max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
        backup_vel_(backup_vel),
        dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
        simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period){

        stuck_left_ = false;
        stuck_right_ = false;
        stuck_left_strafe_ = false;
        stuck_right_strafe_ = false;
        rotating_left_ = false;
        rotating_right_ = false;
        strafe_left_ = false;
        strafe_right_ = false;

        escaping_ = false;
        final_goal_position_valid_ = false;
        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
        return;
    }
    my_local_planner_kernel::~my_local_planner_kernel(){
        return;
    }
    void my_local_planner_kernel::updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
        global_plan_.resize(new_plan.size());
        for (unsigned int i = 0; i < new_plan.size(); ++i){
            global_plan_[i] = new_plan[i];
        }
        if (global_plan_.size() > 0){
            geometry_msgs::PoseStamped& final_goal_pose = global_plan_[global_plan_.size() - 1];
            final_goal_x_ = final_goal_pose.pose.position.x;
            final_goal_y_ = final_goal_pose.pose.position.y;
            final_goal_position_valid_ = true;
        }
        else{
            final_goal_position_valid_ = false;
        }
        if (compute_dists){
            path_map_.resetPathDist();
            goal_map_.resetPathDist();
            path_map_.setTargetCells(costmap_, global_plan_);
            goal_map_.setLocalGoal(costmap_, global_plan_);
            ROS_DEBUG("Path/Goal distance computed");
        }
        return;
    }
    void my_local_planner_kernel::generateTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y, double acc_theta, double impossible_cost, Trajectory& traj){
        // 防止参数调节服务在计算途中修改数据
        boost::mutex::scoped_lock l(configuration_mutex_);

        double x_i = x, y_i = y, theta_i = theta;
        double vx_i = vx, vy_i = vy, vtheta_i = vtheta;

        double vmag = hypot(vx_samp, vy_samp);

        int num_steps;
        if(!heading_scoring_) {
            num_steps = int(std::max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
        }
        else {
            num_steps = int(sim_time_ / sim_granularity_ + 0.5);
        }

        if(num_steps == 0) {
            num_steps = 1;
        }
        double dt = sim_time_ / num_steps;
        double time = 0.0;
        // 重置轨迹
        traj.resetPoints();
        traj.xv_ = vx_samp;
        traj.yv_ = vy_samp;
        traj.thetav_ = vtheta_samp;
        traj.cost_ = -1.0;
        // 各个点的代价
        double path_dist = 0.0;
        double goal_dist = 0.0;
        double occ_cost = 0.0;
        double heading_diff = 0.0;
        for (int i = 0; i < num_steps; i++){
            unsigned int cell_x, cell_y;
            if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
                traj.cost_ = -1.0;
                return;
            }
            double footprint_cost = footprintCost(x_i, y_i, theta_i);
            if (footprint_cost < 0){
                traj.cost_ = -1.0;
                return;
            }
            // 计算代价
            occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y))); // 路径上最大的cost
            if (simple_attractor_){
                goal_dist = std::pow((final_goal_x_ - x_i), 2.0) + std::pow((final_goal_y_ - y_i), 2.0);
            }
            else{
                // 计算三个代价
                bool update_path_and_goal_distances = true;
                if (heading_scoring_){// 只在一个点取三个代价
                    if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt){
                        heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
                    }
                    else{
                        update_path_and_goal_distances = false;
                    }
                }
                // heading_scoring_ == false 则评估每一个点的代价
                if (update_path_and_goal_distances){
                    path_dist = path_map_(cell_x, cell_y).target_dist;
                    goal_dist = goal_map_(cell_x, cell_y).target_dist;
                    if (path_dist >= impossible_cost || goal_dist >= impossible_cost){// 当前点可能是障碍
                        traj.cost_ = -2.0;
                        return;
                    }
                }
            }
            // 添加点
            traj.addPoint(x_i, y_i, theta_i);
            // 更新位姿和速度
            vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
            vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
            vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);
            x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
            y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
            theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

            time += dt;
        }
        double cost = -1.0;

        // cost = path_dist * path_distance_bias_ + goal_dist * goal_distance_bias_ + occ_cost * occdist_scale_ + 0.3 * heading_diff;
        if (!heading_scoring_) {
            cost = path_distance_bias_ * path_dist + goal_dist * goal_distance_bias_ + occdist_scale_ * occ_cost;
        }
        else {
            cost = occdist_scale_ * occ_cost + path_distance_bias_ * path_dist + 0.3 * heading_diff + goal_dist * goal_distance_bias_;
        }
        traj.cost_ = cost;
#ifdef PANNER_DEBUG_MODE
        ROS_INFO("cost: %f, x: %f, y: %f, yaw: %f.", cost, vx_samp, vy_samp, vtheta_samp);
#endif
        return;
    }
    Trajectory my_local_planner_kernel::findBestPath(const geometry_msgs::PoseStamped& global_pose, geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities){
        Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
        Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
        path_map_.resetPathDist();
        goal_map_.resetPathDist();
        std::vector<base_local_planner::Position2DInt> footprint_list = footprint_helper_.getFootprintCells(pos, footprint_spec_, costmap_, true);

        for (unsigned int i = 0; i < footprint_list.size(); ++i) {
            path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
        }
        path_map_.setTargetCells(costmap_, global_plan_);
        goal_map_.setLocalGoal(costmap_, global_plan_);
        ROS_DEBUG("Path/Goal distance computed");

        Trajectory best = createTrajectories(pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], acc_lim_x_, acc_lim_y_, acc_lim_theta_);
        ROS_DEBUG("Trajectories created");

        if(best.cost_ < 0){
            drive_velocities.pose.position.x = 0;
            drive_velocities.pose.position.y = 0;
            drive_velocities.pose.position.z = 0;
            drive_velocities.pose.orientation.w = 1;
            drive_velocities.pose.orientation.x = 0;
            drive_velocities.pose.orientation.y = 0;
            drive_velocities.pose.orientation.z = 0;
        }
        else{
            drive_velocities.pose.position.x = best.xv_;
            drive_velocities.pose.position.y = best.yv_;
            drive_velocities.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, best.thetav_);
            tf2::convert(q, drive_velocities.pose.orientation);
        }
        return best;
    }
    Trajectory my_local_planner_kernel::createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta, double acc_x, double acc_y, double acc_theta){
        double max_vel_x = max_vel_x_, max_vel_theta;// 默认，必须这样写
        double min_vel_x, min_vel_theta;

        if( final_goal_position_valid_ ){
            double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
            // 防止跑过头
            max_vel_x = std::min( max_vel_x, final_goal_dist / sim_time_ );
        }
        if (dwa_){
            max_vel_x = std::max(std::min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
            min_vel_x = std::max(min_vel_x_, vx - acc_x * sim_period_);
            max_vel_theta = std::min(max_vel_th_, vtheta + acc_theta * sim_period_);
            min_vel_theta = std::max(min_vel_th_, vtheta - acc_theta * sim_period_);
        }
        else{
            max_vel_x = std::max(std::min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
            min_vel_x = std::max(min_vel_x_, vx - acc_x * sim_time_);
            max_vel_theta = std::min(max_vel_th_, vtheta + acc_theta * sim_time_);
            min_vel_theta = std::max(min_vel_th_, vtheta - acc_theta * sim_time_);
        }
        double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
        double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

        double vx_samp = min_vel_x, vtheta_samp = min_vel_theta, vy_samp = 0.0;

        Trajectory* best_traj = &traj_one_;
        best_traj->cost_ = -1.0;
        Trajectory* comp_traj = &traj_two_;
        comp_traj->cost_ = -1.0;
        Trajectory* swap = NULL;

        double impossible_cost = path_map_.obstacleCosts();

        if (!escaping_){// 非逃逸状态，正常生成
            for (int i = 0; i < vx_samples_; i++){
                vtheta_samp = 0;
                generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
                if (comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
                    swap = best_traj;
                    best_traj = comp_traj;
                    comp_traj = swap;
                    swap = NULL;
                }
                vtheta_samp = min_vel_theta;
                for (int j = 0; j < vtheta_samples_; j++){
                    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
                    if (comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
                        swap = best_traj;
                        best_traj = comp_traj;
                        comp_traj = swap;
                        swap = NULL;
                    }
                    vtheta_samp += dvtheta;
                }
                vx_samp += dvx;
            }
#ifdef PANNER_DEBUG_MODE
            ROS_INFO("-----------------end of normal--------------------");
#endif
            if (holonomic_robot_){
                vx_samp = 0.1;
                vy_samp = 0.1;
                vtheta_samp = 0.0;
                generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
                if (comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
                    swap = best_traj;
                    best_traj = comp_traj;
                    comp_traj = swap;
                    swap = NULL;
                }
                vx_samp = 0.1;
                vy_samp = -0.1;
                vtheta_samp = 0.0;
                generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
                if (comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
                    swap = best_traj;
                    best_traj = comp_traj;
                    comp_traj = swap;
                    swap = NULL;
                }
            }
        }
        // 原地旋转
        vtheta_samp = min_vel_theta;
        vx_samp = 0.0;
        vy_samp = 0.0;
        double heading_dist = DBL_MAX;

        for (int i = 0; i < vtheta_samples_; i++){
            double vtheta_samp_limited = vtheta_samp > 0 ? std::max(vtheta_samp, min_in_place_vel_th_): std::min(vtheta_samp, -1.0 * min_in_place_vel_th_);
            generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
            if (comp_traj->cost_ >= 0
                && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
                && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
                double x_r, y_r, th_r;
                comp_traj->getEndpoint(x_r, y_r, th_r);
                x_r += heading_lookahead_ * cos(th_r);
                y_r += heading_lookahead_ * sin(th_r);
                unsigned int cell_x, cell_y;
                if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)){
                    double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
                    if (ahead_gdist < heading_dist) {
                        //if we haven't already tried rotating left since we've moved forward
                        if (vtheta_samp < 0 && !stuck_left_) {
                            swap = best_traj;
                            best_traj = comp_traj;
                            comp_traj = swap;
                            swap = NULL;
                            heading_dist = ahead_gdist;
                        }
                        //if we haven't already tried rotating right since we've moved forward
                        else if(vtheta_samp > 0 && !stuck_right_) {
                            swap = best_traj;
                            best_traj = comp_traj;
                            comp_traj = swap;
                            swap = NULL;
                            heading_dist = ahead_gdist;
                        }
                    }
                }
            }
            vtheta_samp += dvtheta;
        }
#ifdef PANNER_DEBUG_MODE
            ROS_INFO("-----------------end of rotate--------------------");
#endif
        // 如果找到合法轨迹
        if (best_traj->cost_ >= 0){
            // 防止原地前后震荡和旋转震荡
            if (!(best_traj->xv_ > 0)){// 停车，不正常
                if (best_traj->thetav_ < 0){
                    if (rotating_right_){
                        stuck_right_ = true;
                    }
                    rotating_right_ = true;
                }
                else if (best_traj->thetav_ > 0){
                    if (rotating_left_){
                        stuck_left_ = true;
                    }
                    rotating_left_ = true;
                }
                else if (best_traj->yv_ > 0){
                    if (strafe_right_){
                        stuck_right_strafe_ = true;
                    }
                    strafe_right_ = true;
                }
                else if (best_traj->yv_ < 0){
                    if (strafe_left_){
                        stuck_left_strafe_ = true;
                    }
                    strafe_left_ = true;
                }
                prev_x_ = x;
                prev_y_ = y;
            }
            double dist = hypot(x - prev_x_, y - prev_y_);
            if (dist > oscillation_reset_dist_) {// 震荡重置
                rotating_left_ = false;
                rotating_right_ = false;
                strafe_left_ = false;
                strafe_right_ = false;
                stuck_left_ = false;
                stuck_right_ = false;
                stuck_left_strafe_ = false;
                stuck_right_strafe_ = false;
            }
            dist = hypot(x - escape_x_, y - escape_y_);
            if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){// 逃逸重置
                escaping_ = false;
            }
#ifdef PANNER_DEBUG_MODE
            ROS_INFO("-----------------found--------------------");
#endif
            return *best_traj;
        }
        // 还没找到的话如果是全约束机器人，找横向运动方法
        if (holonomic_robot_){
            vtheta_samp = min_vel_theta;
            vx_samp = 0.0;
            for (unsigned int i = 0; i < y_vels_.size(); ++i){
                vtheta_samp = 0;
                vy_samp = y_vels_[i];
                generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
                if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
                    double x_r, y_r, th_r;
                    comp_traj->getEndpoint(x_r, y_r, th_r);
                    x_r += heading_lookahead_ * cos(th_r);
                    y_r += heading_lookahead_ * sin(th_r);
                    unsigned int cell_x, cell_y;
                    if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)){
                        double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
                        if (ahead_gdist < heading_dist){
                            //if we haven't already tried strafing left since we've moved forward
                            if (vy_samp > 0 && !stuck_left_strafe_) {
                                swap = best_traj;
                                best_traj = comp_traj;
                                comp_traj = swap;
                                swap = NULL;
                                heading_dist = ahead_gdist;
                            }
                            //if we haven't already tried rotating right since we've moved forward
                            else if(vy_samp < 0 && !stuck_right_strafe_) {
                                swap = best_traj;
                                best_traj = comp_traj;
                                comp_traj = swap;
                                swap = NULL;
                                heading_dist = ahead_gdist;
                            }
                        }
                    }
                }
            }
        }
        if (best_traj->cost_ >= 0){
            if (!(best_traj->xv_ > 0)){// 停车， 不正常
                if (best_traj->thetav_ < 0) {
                    if (rotating_right_){
                        stuck_right_ = true;
                    }
                    rotating_left_ = true;
                } 
                else if(best_traj->thetav_ > 0) {
                    if(rotating_left_){
                        stuck_left_ = true;
                    }
                    rotating_right_ = true;
                } 
                else if(best_traj->yv_ > 0) {
                    if(strafe_right_){
                        stuck_right_strafe_ = true;
                    }
                    strafe_left_ = true;
                } 
                else if(best_traj->yv_ < 0) {
                    if(strafe_left_){
                        stuck_left_strafe_ = true;
                    }
                    strafe_right_ = true;
                }

                //set the position we must move a certain distance away from
                prev_x_ = x;
                prev_y_ = y;
            }
            double dist = hypot(x - prev_x_, y - prev_y_);
            if (dist > oscillation_reset_dist_) {// 震荡重置
                rotating_left_ = false;
                rotating_right_ = false;
                strafe_left_ = false;
                strafe_right_ = false;
                stuck_left_ = false;
                stuck_right_ = false;
                stuck_left_strafe_ = false;
                stuck_right_strafe_ = false;
            }
            dist = hypot(x - escape_x_, y - escape_y_);
            if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){// 逃逸重置
                escaping_ = false;
            }
            return *best_traj;
        }
        // 最后的方法, 倒车
        vtheta_samp = 0.0;
        vx_samp = backup_vel_;
        vy_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
        swap = best_traj;
        best_traj = comp_traj;
        comp_traj = swap;
        swap = NULL;
        double dist = hypot(x - prev_x_, y - prev_y_);
        if (dist > oscillation_reset_dist_) {// 震荡重置
            rotating_left_ = false;
            rotating_right_ = false;
            strafe_left_ = false;
            strafe_right_ = false;
            stuck_left_ = false;
            stuck_right_ = false;
            stuck_left_strafe_ = false;
            stuck_right_strafe_ = false;
        }
        if (!escaping_ && best_traj->cost_ > -2.0){
            escape_x_ = x;
            escape_y_ = y;
            escape_theta_ = theta;
            escaping_ = true;
        }
        dist = hypot(x - escape_x_, y - escape_y_);
        if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){// 逃逸重置
            escaping_ = false;
        }
        // 即使撞到了也倒车
        if(best_traj->cost_ == -1.0){
            best_traj->cost_ = 1.0;
        }
#ifdef PANNER_DEBUG_MODE
            ROS_INFO("-----------------back--------------------");
#endif
        return *best_traj;
    }
};

#endif