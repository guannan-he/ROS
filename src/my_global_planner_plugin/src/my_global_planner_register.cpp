#include <pluginlib/class_list_macros.h>
#include <myCarrot.hpp>
#include <myAStar.hpp>
#include <myDijkstra.hpp>
#include <my_global_planner_common.hpp>
#include <myDijkstraROS.hpp>
#include <myDijkstraKernel.hpp>

PLUGINLIB_EXPORT_CLASS(global_planner::myCarrot, nav_core::BaseGlobalPlanner)
PLUGINLIB_EXPORT_CLASS(global_planner::myAStar, nav_core::BaseGlobalPlanner)
PLUGINLIB_EXPORT_CLASS(global_planner::myDijkstra, nav_core::BaseGlobalPlanner)
PLUGINLIB_EXPORT_CLASS(global_planner::myRosDijkstra, nav_core::BaseGlobalPlanner)

