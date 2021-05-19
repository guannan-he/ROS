/**
 * @file myDijkstraKernel.hpp
 * @brief dijkstra kernel for general use
 * @author guannan-he (guannan-he@outlook.com)
 * @version 1.0
 * @date 2021-05-18
 * 
 * @copyright Copyright (c) {2021}  合肥工业大学-LASIS-何冠男
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-05-18 <td>1.0     <td>guannan-he     <td>内容
 * </table>
 */
#ifndef _MY_DIJKSTRA_KERNEL_
#define _MY_DIJKSTRA_KERNEL_

#include <ros/ros.h>
#include <my_global_planner_common.hpp>
#include <math.h>
#include <string.h>

namespace global_planner{//class and test
    class dijkstraKernel{
    public:
        dijkstraKernel(int xSize, int ySize);
        ~dijkstraKernel();
        /**
         * @brief Set the Start 
         * @param  x                x coord on map
         * @param  y                y coord on map
         */
        void setStart(int x, int y);
        /**
         * @brief Set the Goal 
         * @param  x                x coord on map
         * @param  y                y coord on map
         */
        void setGoal(int x, int y);
        /**
         * @brief reset all arr
         * @param  xSize            xSize
         * @param  ySize            ySize
         */
        void setKernelArr(int xSize, int ySize);
        /**
         * @brief Set the Costmap
         * @param  costmap          costmap array
         * @param  isROS            from ros or pgm
         * @param  allowUnknow      allow explore unknow area
         */
        void setCostmap(const KERNEL_COSTMAP_TYPE* costmap, bool isROS = true, bool allowUnknow = true);
        /**
         * @brief generate path, must be used after goal is changed. recommond use
         * @param  fastStop         paramname
         * @return true 
         * @return false 
         */
        bool dijkstraPath(bool fastStop = true);
        /**
         * @brief Get the Path X 
         * @return float* 
         */
        float* getPathX();
        /**
         * @brief Get the Path Y 
         * @return float* 
         */
        float* getPathY();
        /**
         * @brief Get the Path Len 
         * @return int 
         */
        int getPathLen();
        /**
         * @brief generate path from potential matrix, can also be used when goal is not changed
         * @param  pathLenLim       pathLenLim
         * @param  index            start index
         * @return true 
         * @return false 
         */
        bool generatePath(int pathLenLim, int index);
    private:
        /**
         * @brief convert index to x y
         * @param  index            index
         * @param  x                x
         * @param  y                y
         */
        inline void indexToXY(int& index, int x, int y);
        /**
         * @brief return index in arr from x y value
         * @param  x                x
         * @param  y                y
         * @return int 
         */
        inline int xyToIndex(int x, int y);
        /**
         * @brief push index into current buffer
         * @param  index            index
         */
        inline void pushCurrent(int index);
        /**
         * @brief push index into next buffer
         * @param  index            inmydex
         */
        inline void pushNext(int index);
        /**
         * @brief push overflow index into buffer
         * @param  index            index
         */
        inline void pushOverflow(int index);
        /**
         * @brief must be done at each path calculation, reset buffer and pending array
         * @param  keepCostmap      keepCostmap
         */
        void initCalc(bool keepCostmap = true);
        /**
         * @brief init potential matrix at start and push surrounding index into current buffer
         * @param  index            start index
         * @param  val              potential value at start
         */
        void initCost(int index, float val);
        /**
         * @brief update cell potential and push surrounding cells into buffer
         * @param  index            index
         */
        void updateCell(int index);
        /**
         * @brief update potential field
         * @param  planCycle        planCycle
         * @param  fastStop         fastStop
         * @return true 
         * @return false 
         */
        bool propArr(int planCycle, bool fastStop = true);
        /**
         * @brief update gradient of a cell
         * @param  index            index
         * @return float 
         */
        float gradientCell(int index);
        /**
         * @brief delete all path
         */
        void deletePath();
        /**
         * @brief delete all arrays
         */
        void deleteArr();
        /**
         * @brief delete all buffers
         */
        void deleteBuffs();
        //params
        int xSize_, ySize_, cellCnt_;
        int startIndex_, goalIndex_;
        int pathLen_;
        int obstacleCnt_;
        float thresholdInc_, threshold_;
        float pathStep_;
        //path
        float* pathX_ = NULL;
        float* pathY_ = NULL;
        //arrs
        KERNEL_COSTMAP_TYPE* costmap_ = NULL;
        float* gradientX_ = NULL;
        float* gradientY_ = NULL;
        float* potentialMap_ = NULL;
        bool* pending_ = NULL;
        //buffs
        int* priorityBuffer1_ = NULL;
        int* priorityBuffer2_ = NULL;
        int* priorityBuffer3_ = NULL;
        //pointers& curs
        int* priorBufCurrent_ = NULL;
        int* priorBufNext_ = NULL;
        int* priorBufOverflow_ = NULL;
        int curCurrent_ = 0, curNext_ = 0, curOverflow_ = 0;
    };
    //TODO
    int dijkstraKernelTest(const KERNEL_COSTMAP_TYPE* costmap, int xSize, int ySize, int startX, int startY, int goalX, int goalY, float* pathX, float* pathY, int planCycle){
        return -1;
    }
};

namespace global_planner{//public
    dijkstraKernel::dijkstraKernel(int xSize, int ySize){
        priorityBuffer1_ = new int[PRIORITYBUFSIZE];
        priorityBuffer2_ = new int[PRIORITYBUFSIZE];
        priorityBuffer3_ = new int[PRIORITYBUFSIZE];
        thresholdInc_ = 2 * COST_NEUTRAL;
        pathStep_ = 0.5;
        startIndex_ = goalIndex_ = 0;
        setKernelArr(xSize, ySize);
        return;
    }
    dijkstraKernel::~dijkstraKernel(){
        deletePath();
        deleteArr();
        deleteBuffs();
        return;
    }
    void dijkstraKernel::setStart(int x, int y){
        startIndex_ = x + y * xSize_;
        return;
    }
    void dijkstraKernel::setGoal(int x, int y){
        goalIndex_ = x + y * xSize_;
        return;
    }
    void dijkstraKernel::setKernelArr(int xSize, int ySize){
        xSize_ = xSize;
        ySize_ = ySize;
        cellCnt_ = xSize_ * ySize_;
        deletePath();
        deleteArr();
        costmap_ = new KERNEL_COSTMAP_TYPE[cellCnt_];
        gradientX_ = new float[cellCnt_];
        gradientY_ = new float[cellCnt_];
        potentialMap_ = new float[cellCnt_];
        pending_ = new bool[cellCnt_];
        memset(costmap_, 0, cellCnt_ * sizeof(KERNEL_COSTMAP_TYPE));
        memset(pending_, 0, cellCnt_ * sizeof(bool));
        return;
    }
    void dijkstraKernel::setCostmap(const KERNEL_COSTMAP_TYPE* costmap, bool isROS, bool allowUnknow){
        KERNEL_COSTMAP_TYPE* costmapCur = costmap_;
        if (isROS){
            for (int i = 0; i < ySize_; i++){
                for (int j = 0; j < xSize_; j++, costmapCur++, costmap++){
                    *costmapCur = COST_OBS;
                    int val = *costmap;
                    if (val < COST_OBS_ROS){
                        val = COST_NEUTRAL + COST_FACTOR * val;
                        if (val >= COST_OBS){
                            val = COST_OBS - 1;
                        }
                        *costmapCur = val;
                    }
                    else if(val == COST_UNKNOWN_ROS && allowUnknow){
                        val = COST_OBS - 1;
                        *costmapCur = val;
                    }
                }
            }
        }
        else{
            for (int i = 0; i < ySize_; i++){
                for (int j = 0; j < xSize_; j++, costmapCur++, costmap++){
                    *costmapCur = COST_OBS;
                    if (i < 7 || i > ySize_ - 8 || j < 7 || j > xSize_ - 8){
                        continue;
                    }
                    int val = *costmap;
                    if (val < COST_OBS_SERVER){
                        val = COST_NEUTRAL + 2.5 * COST_FACTOR * val;
                        if (val >= COST_OBS){
                            val = COST_OBS - 1;
                        }
                        *costmapCur = val;
                    }
                    else if(val == COST_UNKNOWN_ROS){
                        val = COST_OBS - 1;
                        *costmapCur = val;
                    }
                }
            }
        }
        return;
    }
    float* dijkstraKernel::getPathX(){
        return pathX_;
    }
    float* dijkstraKernel::getPathY(){
        return pathY_;
    }
    int dijkstraKernel::getPathLen(){
        return pathLen_;
    }
    bool dijkstraKernel::dijkstraPath(bool fastStop){
        initCalc(true);
        propArr(std::max(xSize_, ySize_), xSize_ + ySize_);
        return generatePath(cellCnt_ / 2, startIndex_);
    }
    bool dijkstraKernel::generatePath(int pathLenLim, int index){
        deletePath();
        pathX_ = new float[pathLenLim];
        pathY_ = new float[pathLenLim];
        float dx = 0.0, dy = 0.0;
        while (pathLen_ < pathLenLim){
            if (index < xSize_ || index > cellCnt_ - xSize_){
                deletePath();
                return false;
            }
            int nearIndex = std::min(cellCnt_ - 1, index + (int)round(dx) + (int)(xSize_ * round(dy)));
            if (potentialMap_[nearIndex] < COST_NEUTRAL){
                pathX_[pathLen_] = (float)(goalIndex_ % xSize_);
                pathY_[pathLen_] = (float)(goalIndex_ / xSize_);
                pathLen_++;
                return true;
            }
            pathX_[pathLen_] = index % xSize_ + dx;
            pathY_[pathLen_] = index / xSize_ + dy;
            bool oscillation_detected = false;
            if (pathLen_ > 1 && pathX_[pathLen_] == pathX_[pathLen_ - 2] && pathY_[pathLen_] == pathY_[pathLen_ - 2]){
                ROS_WARN("oscillation_detected");
                oscillation_detected = true;
            }
            int upperIndex = index - xSize_, downIndex = index + xSize_;
            if (//检测到震荡或者周围有障碍物，换一个格
                potentialMap_[index] >= POT_HIGH ||
                potentialMap_[index + 1] >= POT_HIGH ||
                potentialMap_[index - 1] >= POT_HIGH ||
                potentialMap_[upperIndex] >= POT_HIGH ||
                potentialMap_[upperIndex + 1] >= POT_HIGH ||
                potentialMap_[upperIndex - 1] >= POT_HIGH ||
                potentialMap_[downIndex] >= POT_HIGH ||
                potentialMap_[downIndex + 1] >= POT_HIGH ||
                potentialMap_[downIndex - 1] >= POT_HIGH ||
                oscillation_detected){
                ROS_WARN("obstacle nearby or oscillation_detected, fixing.");
                int minIndex = index;
                float minPotential = potentialMap_[index];
                if (potentialMap_[index - 1] < minPotential){
                    minIndex = index - 1;
                    minPotential = potentialMap_[index - 1];
                }
                if (potentialMap_[index + 1] < minPotential){
                    minIndex = index + 1;
                    minPotential = potentialMap_[index + 1];
                }
                if (potentialMap_[upperIndex - 1] < minPotential){
                    minIndex = upperIndex - 1;
                    minPotential = potentialMap_[upperIndex - 1];
                }
                if (potentialMap_[upperIndex] < minPotential){
                    minIndex = upperIndex;
                    minPotential = potentialMap_[upperIndex];
                }
                if (potentialMap_[upperIndex + 1] < minPotential){
                    minIndex = upperIndex + 1;
                    minPotential = potentialMap_[upperIndex + 1];
                }
                if (potentialMap_[downIndex - 1] < minPotential){
                    minIndex = downIndex - 1;
                    minPotential = potentialMap_[downIndex - 1];
                }
                if (potentialMap_[downIndex] < minPotential){
                    minIndex = downIndex;
                    minPotential = potentialMap_[downIndex];
                }
                if (potentialMap_[downIndex + 1] < minPotential){
                    minIndex = downIndex + 1;
                    minPotential = potentialMap_[downIndex + 1];
                }
                index = minIndex;
                dx = 0.0, dy = 0.0;
                if (potentialMap_[index] >= POT_HIGH){
                    ROS_WARN("fix fail.");
                    deletePath();
                    return false;
                }
            }
            else{
                gradientCell(index);
                gradientCell(index + 1);
                gradientCell(downIndex);
                gradientCell(downIndex + 1);
                float x1 = (1.0 - dx) * gradientX_[index] + dx * gradientX_[index + 1];
                float x2 = (1.0 - dx) * gradientX_[downIndex] + dx * gradientX_[downIndex + 1];
                float y1 = (1.0 - dx) * gradientY_[index] + dx * gradientY_[index + 1];
                float y2 = (1.0 - dx) * gradientY_[downIndex] + dx * gradientY_[downIndex + 1];
                float x = (1.0 - dy) * x1 + dy * x2;
                float y = (1.0 - dy) * y1 + dy * y2;
                if (x == 0.0 && y == 0.0){
                    ROS_WARN("zero gradient.");
                    deletePath();
                    return false;
                }
                float stepSize = pathStep_ / hypot(x, y);
                dx += x * stepSize;
                dy += y * stepSize;
                if (dx > 1.0){
                    dx -= 1.0;
                    index += 1;
                }
                if (dx < -1.0){
                    dx += 1.0;
                    index -= 1;
                }
                if (dy > 1.0){
                    dy -= 1.0;
                    index += xSize_;
                }
                if (dy < -1.0){
                    dy += 1.0;
                    index -= xSize_;
                }
            }
            pathLen_++;
        }
        ROS_WARN("path too long.");
        return false;
    }
};

namespace global_planner{//private
    void dijkstraKernel::pushCurrent(int index){
        if (index >= 0 && index < cellCnt_ && !pending_[index] && costmap_[index] < COST_OBS && curCurrent_ < PRIORITYBUFSIZE){
            priorBufCurrent_[curCurrent_++] = index;
            pending_[index] = true;
        }
        return;
    }
    void dijkstraKernel::pushNext(int index){
        if (index >= 0 && index < cellCnt_ && !pending_[index] && costmap_[index] < COST_OBS && curNext_ < PRIORITYBUFSIZE){
            priorBufNext_[curNext_++] = index;
            pending_[index] = true;
        }
        return;
    }
    void dijkstraKernel::pushOverflow(int index){
        if (index >= 0 && index < cellCnt_ && !pending_[index] && costmap_[index] < COST_OBS && curOverflow_ < PRIORITYBUFSIZE){
            priorBufOverflow_[curOverflow_++] = index;
            pending_[index] = true;
        }
        return;
    }
    int dijkstraKernel::xyToIndex(int x, int y){
        return x + y * xSize_;
    }
    void dijkstraKernel::indexToXY(int& index, int x, int y){
        x = index & xSize_;
        y = index / xSize_;
        return;
    }
    void dijkstraKernel::deletePath(){
        if (pathX_ != NULL){
            delete[] pathX_;
            pathX_ = NULL;
        }
        if (pathY_ != NULL){
            delete[] pathY_;
            pathY_ = NULL;
        }
        pathLen_ = 0;
        return;
    }
    void dijkstraKernel::deleteArr(){
        if (costmap_ != NULL){
            delete[] costmap_;
            costmap_ = NULL;
        }
        if (potentialMap_ != NULL){
            delete[] potentialMap_;
            potentialMap_ = NULL;
        }
        if (gradientX_ != NULL){
            delete[] gradientX_;
            gradientX_ = NULL;
        }
        if (gradientY_ != NULL){
            delete[] gradientY_;
            gradientY_ = NULL;
        }
        if (pending_ != NULL){
            delete[] pending_;
            pending_ = NULL;
        }
        return;
    }
    void dijkstraKernel::deleteBuffs(){
        if (priorityBuffer1_ != NULL){
            delete[] priorityBuffer1_;
            priorityBuffer1_ = NULL;
        }
        if (priorityBuffer2_ != NULL){
            delete[] priorityBuffer2_;
            priorityBuffer2_ = NULL;
        }
        if (priorityBuffer3_ != NULL){
            delete[] priorityBuffer3_;
            priorityBuffer3_ = NULL;
        }
        return;
    }
    void dijkstraKernel::initCost(int index, float val){
        potentialMap_[index] = val;
        pushCurrent(index + 1);
        pushCurrent(index - 1);
        pushCurrent(index + xSize_);
        pushCurrent(index - xSize_);
        return;
    }
    void dijkstraKernel::initCalc(bool keepCostmap){
        for (int i = 0; i < cellCnt_; i++){
            potentialMap_[i] = POT_HIGH;
            gradientX_[i] = 0.0;
            gradientY_[i] = 0.0;
        }
        if (!keepCostmap){
            memset(costmap_, COST_NEUTRAL, cellCnt_ * sizeof(KERNEL_COSTMAP_TYPE));
        }
        KERNEL_COSTMAP_TYPE* costmapCur;
        // 处理边框, 可以确保不访问越界
        costmapCur = costmap_;
        for (int i = 0; i < xSize_; i++){
            *costmapCur++ = COST_OBS;
        }
        costmapCur = costmap_ + (ySize_ - 1) * xSize_;
        for (int i = 0; i < xSize_; i++){
            *costmapCur++ = COST_OBS;
        }
        costmapCur = costmap_;
        for (int i = 0; i < ySize_; i += xSize_){
            *costmapCur++ = COST_OBS;
        }
        costmapCur = costmap_ + xSize_ - 1;
        for (int i = 0; i < ySize_; i += xSize_){
            *costmapCur++ = COST_OBS;
        }
        priorBufCurrent_ = priorityBuffer1_;
        priorBufNext_ = priorityBuffer2_;
        priorBufOverflow_ = priorityBuffer3_;
        curCurrent_ = curNext_ = curOverflow_ = 0;
        memset(pending_, 0, cellCnt_ * sizeof(bool));
        initCost(goalIndex_, 0.0);
        //重置阈值
        threshold_ = COST_OBS;
        costmapCur = costmap_;
        obstacleCnt_ = 0;
        for (int i = 0; i < cellCnt_; i++){
            if (*costmapCur >= COST_OBS){
                obstacleCnt_++;
            }
        }
        return;
    }
    void dijkstraKernel::updateCell(int index){
        if (costmap_[index] >= COST_OBS){//不刷新障碍
            return;
        }
        float upVal, downVal, leftVal, rightVal;
        upVal = potentialMap_[index - xSize_];
        downVal = potentialMap_[index + xSize_];
        leftVal = potentialMap_[index - 1];
        rightVal = potentialMap_[index + 1];
        //E 插值，计算势能场
        float ta, tc;
        tc = std::min(leftVal, rightVal);
        ta = std::min(upVal, downVal);
        float hf = (float)costmap_[index];
        float dc = tc - ta;
        if (dc < 0){
            dc = -dc;
            ta = tc;
        }
        float potential;
        if (dc >= hf){
            potential = ta + hf;
        }
        else{
            float d = dc / hf;
            float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
            potential = ta + hf * v;
        }
        if (potential < potentialMap_[index]){//拉低相邻势能
            float upValE, downValE, leftValE, rightValE;
            upValE = INVSQRT2 * (float)costmap_[index - xSize_];
            downValE = INVSQRT2 * (float)costmap_[index + xSize_];
            leftValE = INVSQRT2 * (float)costmap_[index - 1];
            rightValE = INVSQRT2 * (float)costmap_[index + 1];
            potentialMap_[index] = potential;
            if (potential < threshold_){
                if (potential + upValE < upVal){
                    pushNext(index - xSize_);
                }
                if (potential + downValE < downVal){
                    pushNext(index + xSize_);
                }
                if (potential + leftValE < leftVal){
                    pushNext(index - 1);
                }
                if (potential + rightValE < rightVal){
                    pushNext(index + 1);
                }
            }
            else{
                if (potential + upValE < upVal){
                    pushOverflow(index - xSize_);
                }
                if (potential + downValE < downVal){
                    pushOverflow(index + xSize_);
                }
                if (potential + leftValE < leftVal){
                    pushOverflow(index - 1);
                }
                if (potential + rightValE < rightVal){
                    pushOverflow(index + 1);
                }
            }
        }
        return;
    }
    float dijkstraKernel::gradientCell(int index){
        if (gradientX_[index] + gradientY_[index] > 0.0){
            return 1.0;
        }
        if (index < xSize_ || index > cellCnt_ - xSize_){
            return 0.0;
        }
        float potential = potentialMap_[index];
        float dx = 0.0, dy = 0.0;
        if (potential >= POT_HIGH){
            if (potentialMap_[index - 1] < POT_HIGH){
                dx = -COST_OBS;
            }
            else if (potentialMap_[index + 1] < POT_HIGH){
                dx = COST_OBS;
            }
            if (potentialMap_[index - xSize_] < POT_HIGH){
                dy = -COST_OBS;
            }
            else if (potentialMap_[index + xSize_] < POT_HIGH){
                dy = COST_OBS;
            }
        }
        else{
            if (potentialMap_[index - 1] < POT_HIGH){
                dx += potentialMap_[index - 1] - potential;
            }
            if (potentialMap_[index + 1] < POT_HIGH){
                dx += potential - potentialMap_[index + 1];
            }
            if (potentialMap_[index - xSize_] < POT_HIGH){
                dy += potentialMap_[index - xSize_] - potential;
            }
            if (potentialMap_[index + xSize_] < POT_HIGH){
                dy += potential - potentialMap_[index + xSize_];
            }
        }
        float norm = hypot(dx, dy);
        if (norm > 0.0){
            norm = 1.0 / norm;
            gradientX_[index] = dx * norm;
            gradientY_[index] = dy * norm;
        }
        return norm;
    }
    bool dijkstraKernel::propArr(int planCycle, bool fastStop){
        while (planCycle-- > 0){
            if (curCurrent_ == 0 && curNext_ == 0){
                // ROS_INFO("found best plan.");
                break;
            }
            int* bufferPointer = priorBufCurrent_;
            int bufferCellCnt = curCurrent_;
            while (bufferCellCnt-- > 0){
                pending_[*(bufferPointer++)] = false;
            }
            bufferPointer = priorBufCurrent_;
            bufferCellCnt = curCurrent_;
            while (bufferCellCnt-- > 0){
                updateCell(*bufferPointer++);
            }
            curCurrent_ = curNext_;
            curNext_ = 0;
            bufferPointer = priorBufCurrent_;
            priorBufCurrent_ = priorBufNext_;
            priorBufNext_ = bufferPointer;
            if (curCurrent_ == 0){
                threshold_ += thresholdInc_;
                curCurrent_ = curOverflow_;
                curOverflow_ = 0;
                bufferPointer = priorBufCurrent_;
                priorBufCurrent_ = priorBufOverflow_;
                priorBufOverflow_ = bufferPointer;
            }
            if (fastStop && potentialMap_[startIndex_] < POT_HIGH){
                // ROS_INFO("found plan and fast stop.");
                break;
            }
        }
        if (planCycle > 0){
            return true;
        }
        // ROS_WARN("no plan within cycles.");
        return false;
    }
};

#endif