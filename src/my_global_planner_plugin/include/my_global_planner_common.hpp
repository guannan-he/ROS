#ifndef _MY_GLOBAL_PLANNER_COMMON_
#define _MY_GLOBAL_PLANNER_COMMON_

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