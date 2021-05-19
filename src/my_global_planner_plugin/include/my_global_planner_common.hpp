#ifndef _MY_GLOBAL_PLANNER_COMMON_
#define _MY_GLOBAL_PLANNER_COMMON_

#define KERNEL_COSTMAP_TYPE unsigned char

#define COST_UNKNOWN_ROS 255
#define COST_OBS 254
#define COST_OBS_ROS 253
#define COST_OBS_SERVER 100
#define COST_NEUTRAL 50
#define COST_FACTOR 0.8
#define POT_HIGH 1.0e10
#define PRIORITYBUFSIZE 10000
#define INVSQRT2 0.707106781

namespace global_planner{
    const double infVal = 1.0e10;

    struct aSrarNode{
        double cost = infVal;
        int index = 0;
    };

    bool operator <(const global_planner::aSrarNode& x, const global_planner::aSrarNode& y) {
        return x.cost < y.cost;
    }

    struct dijkstraNode{
        double cost = infVal;
        int index = 0;
    };

    bool operator <(const global_planner::dijkstraNode& x, const global_planner::dijkstraNode& y) {
        return x.cost < y.cost;
    }
};

#endif