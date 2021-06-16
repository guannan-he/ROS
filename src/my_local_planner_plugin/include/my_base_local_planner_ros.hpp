/**
 * @file my_base_local_planner_ros.hpp
 * @brief 
 * @author guannan-he (guannan-he@outlook.com)
 * @version 1.0
 * @date 2021-06-15
 * 
 * @copyright Copyright (c) {2021}  合肥工业大学-LASIS-何冠男
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-06-15 <td>1.0     <td>guannan-he     <td>内容
 * </table>
 */

/*  由 nav_core::BaseLocalPlanner 定义接口，由 move_base 调用
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;
    virtual bool isGoalReached() = 0;
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;
    virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;
*/
#ifndef _MY_BASE_LOCAL_PLANNER_ROS_
#define _MY_BASE_LOCAL_PLANNER_ROS_

// 是否使用原版内核
// #define USE_OFFICAL_BASE_PLANNER_KERNEL

#ifdef HAVE_SYS_TIME_H 
    #include <sys/time.h>
#endif

#ifdef USE_OFFICAL_BASE_PLANNER_KERNEL
    #include <base_local_planner/trajectory_planner.h>
    #define BASE_PLANNER_TYPE TrajectoryPlanner
#else
    #include <my_base_local_planner.hpp>
    #define BASE_PLANNER_TYPE my_local_planner_kernel
#endif


#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <base_local_planner/map_grid_visualizer.h>
#include <base_local_planner/planar_laser_scan.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/buffer.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>


#include <base_local_planner/odometry_helper_ros.h>

#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>

// 类定义
namespace base_local_planner{
    class myBaseLocalPlannerROS : public nav_core::BaseLocalPlanner {
        public:
        /**
         * @brief Construct a new my Base Local Planner R O S object
         */
        myBaseLocalPlannerROS();

        /**
         * @brief Construct a new my Base Local Planner R O S object
         * @param  name                            name
         * @param  tf                              获取变换
         * @param  costmap_ros                     局部代价图
         */
        myBaseLocalPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief 默认初始化函数，由 move_base 调用
         * @param  name                            name
         * @param  tf                              获取变换
         * @param  costmap_ros                     局部代价图
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief Destroy the my Base Local Planner R O S object
         */
        ~myBaseLocalPlannerROS();

        /**
         * @brief 输出控制命令，由 move_base 调用
         * @param  cmd_vel                         paramname
         * @return true 成功
         * @return false 失败
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief Set the Plan object
         * @param  plan                            全局路径点
         * @return true 
         * @return false 
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief 是否到达目标点
         * @return true 
         * @return false 
         */
        bool isGoalReached();

        /**
         * @brief 检测轨迹是否合法
         * @param  vx_samp                         vx_samp
         * @param  vy_samp                         vy_samp
         * @param  vtheta_samp                     vtheta_samp
         * @param  update_map                      是否更新局部地图
         * @return true 
         * @return false 
         */
        bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true);

        /**
         * @brief 为轨迹评分
         * @param  vx_samp                         vx_samp
         * @param  vy_samp                         vy_samp
         * @param  vtheta_samp                     vtheta_samp
         * @param  update_map                      是否更新局部地图
         * @return double 
         */
        double scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true);

        /**
         * @brief 是否初始化
         * @return true 
         * @return false 
         */
        bool isInitialized();

        /**
         * @brief Get the Planner object
         * @return BASE_PLANNER_TYPE* 
         */
        BASE_PLANNER_TYPE* getPlanner() const;

        private:

        /**
         * @brief 参数调整服务回调函数
         * @param  config                          config
         * @param  level                           level
         */
        void reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level);

        /**
         * @brief 原地旋转到目标方位
         * @param  global_pose                     global_pose
         * @param  robot_vel                       robot_vel
         * @param  goal_th                         goal_th
         * @param  cmd_vel                         cmd_vel
         * @return true 
         * @return false 
         */
        bool rotateToGoal(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);

        /**
         * @brief 考虑加速度限制停止
         * @param  global_pose                     global_pose
         * @param  robot_vel                       robot_vel
         * @param  cmd_vel                         cmd_vel
         * @return true 
         * @return false 
         */
        bool stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel);
        
        /**
         * @brief 从参数服务器获取可用 Y 速度
         * @param  node                            NodeHandle
         * @return std::vector<double> 
         */
        std::vector<double> loadYVels(ros::NodeHandle node);

        /**
         * @brief 获取符号
         * @param  x                               x
         * @return double 
         */
        inline double sign(double x);

        WorldModel* world_model_; ///< @brief 世界模型
        BASE_PLANNER_TYPE* tc_; ///< @brief kernel
        costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
        costmap_2d::Costmap2D* costmap_; ///< @brief The costmap the controller will use
        MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function
        tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds
        std::string global_frame_; ///< @brief The frame in which the controller will run
        double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
        nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
        std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
        double rot_stopped_velocity_, trans_stopped_velocity_; ///< @brief 停止速度下限
        double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_; ///< @brief 容忍误差
        std::vector<geometry_msgs::PoseStamped> global_plan_; ///< @brief 全局规划
        bool prune_plan_; ///< @brief 是否裁减全局规划
        boost::recursive_mutex odom_lock_; ///< @brief odom 线程锁

        double max_vel_th_, min_vel_th_; ///< @brief 纵向速度限制
        double acc_lim_x_, acc_lim_y_, acc_lim_theta_; ///< @brief 加速度限制
        double sim_period_; ///< @brief 仿真时间
        bool rotating_to_goal_; ///< @brief 到目标后是否旋转
        bool reached_goal_; ///< @brief 是否到达目标
        bool latch_xy_goal_tolerance_, xy_tolerance_latch_; ///< @brief 横向纵向误差锁

        ros::Publisher g_plan_pub_, l_plan_pub_; ///< @brief 全局与局部轨迹规划发布

        dynamic_reconfigure::Server<BaseLocalPlannerConfig>* dsrv_; ///< @brief 开参数调节服务
        base_local_planner::BaseLocalPlannerConfig default_config_; ///< @brief 储存参数
        bool setup_; ///< @brief 参数是否获取


        bool initialized_; ///< @brief kernel就绪
        base_local_planner::OdometryHelperRos odom_helper_; ///< @brief 获取odom

        std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief 获取轮廓
    };
};

// 辅助函数
namespace base_local_planner{
    void myBaseLocalPlannerROS::reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level){
        if (setup_ && config.restore_defaults) {
            config = default_config_;
            //Avoid looping
            config.restore_defaults = false;
        }
        if (!setup_) {
            default_config_ = config;
            setup_ = true;
        }
        tc_->reconfigure(config);
        reached_goal_ = false;
        return;
    }
    std::vector<double> myBaseLocalPlannerROS::loadYVels(ros::NodeHandle node){
        std::vector<double> y_vels;
        std::string y_vel_list;
        if(node.getParam("y_vels", y_vel_list)){
            typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
            boost::char_separator<char> sep("[], ");
            tokenizer tokens(y_vel_list, sep);

            for(tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++){
                y_vels.push_back(atof((*i).c_str()));
            }
        }
        else{
            //if no values are passed in, we'll provide defaults
            y_vels.push_back(-0.3);
            y_vels.push_back(-0.1);
            y_vels.push_back(0.1);
            y_vels.push_back(0.3);
        }
        return y_vels;
    }
    inline double myBaseLocalPlannerROS::sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
    }
    bool myBaseLocalPlannerROS::isGoalReached(){
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        return reached_goal_; 
    }
    bool myBaseLocalPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
        geometry_msgs::PoseStamped global_pose;
        if(costmap_ros_->getRobotPose(global_pose)){
            if(update_map){
                //we need to give the planne some sort of global plan, since we're only checking for legality
                //we'll just give the robots current position
                std::vector<geometry_msgs::PoseStamped> plan;
                plan.push_back(global_pose);
                tc_->updatePlan(plan, true);
            }

            //copy over the odometry information
            nav_msgs::Odometry base_odom;
            {
                boost::recursive_mutex::scoped_lock lock(odom_lock_);
                base_odom = base_odom_;
            }

            return tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation), base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y, base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

        }
        ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
        return false;
    }
    double myBaseLocalPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
        geometry_msgs::PoseStamped global_pose;
        if(costmap_ros_->getRobotPose(global_pose)){
            if(update_map){
                //we need to give the planne some sort of global plan, since we're only checking for legality
                //we'll just give the robots current position
                std::vector<geometry_msgs::PoseStamped> plan;
                plan.push_back(global_pose);
                tc_->updatePlan(plan, true);
            }

            //copy over the odometry information
            nav_msgs::Odometry base_odom;
            // 加作用域
            {
                boost::recursive_mutex::scoped_lock lock(odom_lock_);
                base_odom = base_odom_;
            }

            return tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation), base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y, base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

        }
        ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
        return -1.0;
    }
    bool myBaseLocalPlannerROS::isInitialized(){
        return initialized_;
    }
    BASE_PLANNER_TYPE* myBaseLocalPlannerROS::getPlanner() const {
        return tc_;
    }
    bool myBaseLocalPlannerROS::rotateToGoal(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
        double yaw = tf2::getYaw(global_pose.pose.orientation);
        double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
        double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_, std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_, std::min(-1.0 * min_in_place_vel_th_, ang_diff));
        //take the acceleration limits of the robot into account
        double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
        double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

        v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);
        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

        v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));
        v_theta_samp = v_theta_samp > 0.0 ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp )) : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

        bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw, robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, 0.0, 0.0, v_theta_samp);
        if(valid_cmd){
            cmd_vel.angular.z = v_theta_samp;
            return true;
        }
        cmd_vel.angular.z = 0.0;
        return false;
    }
    bool myBaseLocalPlannerROS::stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel){
        double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim_x_ * sim_period_));
        double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim_y_ * sim_period_));
        double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
        double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));
        double yaw = tf2::getYaw(global_pose.pose.orientation);
        bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw, robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, vx, vy, vth);
        if(valid_cmd){
            ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
            cmd_vel.linear.x = vx;
            cmd_vel.linear.y = vy;
            cmd_vel.angular.z = vth;
            return true;
        }
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }
};

// 核心函数
namespace base_local_planner{
    myBaseLocalPlannerROS::myBaseLocalPlannerROS():
    world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), costmap_(NULL), dsrv_(NULL),
    setup_(false), initialized_(false), odom_helper_("odom"){
        return;
    }
    myBaseLocalPlannerROS::myBaseLocalPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros):
    world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), costmap_(NULL), dsrv_(NULL),
    setup_(false), initialized_(false), odom_helper_("odom"){
        initialize(name, tf, costmap_ros);
        return;
    }
    void myBaseLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
        if (initialized_){
            ROS_WARN("This planner has already been initialized, doing nothing");
            return;
        }
        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        rot_stopped_velocity_ = 1e-2;
        trans_stopped_velocity_ = 1e-2;
        double sim_time, sim_granularity, angular_sim_granularity;
        int vx_samples, vtheta_samples;
        double path_distance_bias, goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
        bool holonomic_robot, dwa, simple_attractor, heading_scoring;
        double heading_scoring_timestep;
        double max_vel_x, min_vel_x;
        double backup_vel;
        double stop_time_buffer;
        std::string world_model_type;
        rotating_to_goal_ = false;

        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();
        private_nh.param("prune_plan", prune_plan_, true);
        private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
        private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
        private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
        private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);
        private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);
        private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

        if(private_nh.hasParam("acc_limit_x")){
            ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");
        }
        if(private_nh.hasParam("acc_limit_y")){
            ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");
        }
        if(private_nh.hasParam("acc_limit_th")){
            ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");
        }
        std::string controller_frequency_param_name;
        if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)){
            sim_period_ = 0.05;
        }
        else{
            double controller_frequency = 0;
            private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
            if(controller_frequency > 0){
                sim_period_ = 1.0 / controller_frequency;
            }
            else{
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                sim_period_ = 0.05;
            }
        }
        ROS_INFO("Sim period is set to %.2f", sim_period_);
        private_nh.param("sim_time", sim_time, 1.0);
        private_nh.param("sim_granularity", sim_granularity, 0.025);
        private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
        private_nh.param("vx_samples", vx_samples, 3);
        private_nh.param("vtheta_samples", vtheta_samples, 20);

        path_distance_bias = nav_core::loadParameterWithDeprecation(private_nh, "path_distance_bias", "pdist_scale", 0.6);
        goal_distance_bias = nav_core::loadParameterWithDeprecation(private_nh, "goal_distance_bias", "gdist_scale", 0.6);
        if (private_nh.hasParam("pdist_scale") & !private_nh.hasParam("path_distance_bias")){
            private_nh.setParam("path_distance_bias", path_distance_bias);
        }
        if (private_nh.hasParam("gdist_scale") & !private_nh.hasParam("goal_distance_bias")){
            private_nh.setParam("goal_distance_bias", goal_distance_bias);
        }
        private_nh.param("occdist_scale", occdist_scale, 0.01);

        bool meter_scoring;
        if ( ! private_nh.hasParam("meter_scoring")) {
            ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
        }
        else {
            private_nh.param("meter_scoring", meter_scoring, false);
            if(meter_scoring) {
                //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
                double resolution = costmap_->getResolution();
                goal_distance_bias *= resolution;
                path_distance_bias *= resolution;
            }
            else {
                ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settings robust against changes of costmap resolution.");
            }
        }

        private_nh.param("heading_lookahead", heading_lookahead, 0.325);
        private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
        private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
        private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
        private_nh.param("holonomic_robot", holonomic_robot, true);
        private_nh.param("max_vel_x", max_vel_x, 0.5);
        private_nh.param("min_vel_x", min_vel_x, 0.1);
        double max_rotational_vel;
        private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
        max_vel_th_ = max_rotational_vel;
        min_vel_th_ = -1.0 * max_rotational_vel;

        min_in_place_vel_th_ = nav_core::loadParameterWithDeprecation(private_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
        reached_goal_ = false;
        backup_vel = -0.1;
        if(private_nh.getParam("backup_vel", backup_vel)){
            ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");
        }
        //if both backup_vel and escape_vel are set... we'll use escape_vel
        private_nh.getParam("escape_vel", backup_vel);

        if(backup_vel >= 0.0){
            ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");
        }
        private_nh.param("world_model", world_model_type, std::string("costmap"));
        private_nh.param("dwa", dwa, true);
        private_nh.param("heading_scoring", heading_scoring, false);
        private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

        simple_attractor = false;
        double min_pt_separation, max_obstacle_height, grid_resolution;
        private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
        private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
        private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
        private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);
        ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
        world_model_ = new CostmapModel(*costmap_);
        std::vector<double> y_vels = loadYVels(private_nh);

        footprint_spec_ = costmap_ros_->getRobotFootprint();

        tc_ = new BASE_PLANNER_TYPE(*world_model_, *costmap_, footprint_spec_,
            acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, path_distance_bias,
            goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
            max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
            dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

        map_viz_.initialize(name, global_frame_, boost::bind(&BASE_PLANNER_TYPE::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
        initialized_ = true;

        dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = boost::bind(&myBaseLocalPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // ROS_WARN("%s", robot_base_frame_.c_str());
        // ROS_WARN("%s", global_frame_.c_str());

        return;
    }
    myBaseLocalPlannerROS::~myBaseLocalPlannerROS() {
        //make sure to clean things up
        if(dsrv_ != NULL){
            delete dsrv_;
        }
        if(tc_ != NULL){
            delete tc_;
        }
        if(world_model_ != NULL){
            delete world_model_;
        }
        return;
    }
    bool myBaseLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        if (!isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped global_pose;
        if (!costmap_ros_->getRobotPose(global_pose)) {
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }
        if(prune_plan_){
            prunePlan(global_pose, transformed_plan, global_plan_);
        }

        if(transformed_plan.empty()){
            return false;
        }

        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = robot_base_frame_;
        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        const geometry_msgs::PoseStamped& goal_point = transformed_plan.back();
        //we assume the global goal is the last point in the global plan
        const double goal_x = goal_point.pose.position.x;
        const double goal_y = goal_point.pose.position.y;
        const double yaw = tf2::getYaw(goal_point.pose.orientation);
        double goal_th = yaw;

        if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)){
            //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
            //just rotate in place
            if (latch_xy_goal_tolerance_) {
                xy_tolerance_latch_ = true;
            }
            double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
            if (fabs(angle) <= yaw_goal_tolerance_) {
                //set the velocity command to zero
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                rotating_to_goal_ = false;
                xy_tolerance_latch_ = false;
                reached_goal_ = true;
            }
            else{
                //we need to call the next two lines to make sure that the trajectory
                //planner updates its path distance and goal distance grids
                tc_->updatePlan(transformed_plan);
                Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
                map_viz_.publishCostCloud(costmap_);

                //copy over the odometry information
                nav_msgs::Odometry base_odom;
                odom_helper_.getOdom(base_odom);
                if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
                    if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
                        return false;
                    }
                    //if we're stopped... then we want to rotate to goal
                    else{
                        //set this so that we know its OK to be moving
                        rotating_to_goal_ = true;
                        if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
                            return false;
                        }
                    }
                }
            }
            //publish an empty plan because we've reached our goal position
            publishPlan(transformed_plan, g_plan_pub_);
            publishPlan(local_plan, l_plan_pub_);

            //we don't actually want to run the controller when we're just rotating to goal
            return true;
        }
        tc_->updatePlan(transformed_plan);
        //compute what trajectory to drive along
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);
        //pass along drive commands
        cmd_vel.linear.x = drive_cmds.pose.position.x;
        cmd_vel.linear.y = drive_cmds.pose.position.y;
        cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);
        if (path.cost_ < 0) {
            ROS_DEBUG_NAMED("trajectory_planner_ros",
                "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
            local_plan.clear();
            publishPlan(transformed_plan, g_plan_pub_);
            publishPlan(local_plan, l_plan_pub_);
            return false;
        }
#ifdef PANNER_DEBUG_MODE
            ROS_INFO("A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
#endif
        for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            path.getPoint(i, p_x, p_y, p_th);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = p_x;
            pose.pose.position.y = p_y;
            pose.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, pose.pose.orientation);
            local_plan.push_back(pose);
        }
        //publish information to the visualizer
        publishPlan(transformed_plan, g_plan_pub_);
        publishPlan(local_plan, l_plan_pub_);
        return true;
    }
    bool myBaseLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        if (!isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        global_plan_.clear();
        global_plan_ = plan;
        xy_tolerance_latch_ = false;
        //reset the at goal flag
        reached_goal_ = false;
        return true;
    }
};

#endif