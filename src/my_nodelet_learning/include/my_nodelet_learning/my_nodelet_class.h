#ifndef _MY_NODELET_CLASS_
#define _MY_NODELET_CLASS_

// API:
// http://wiki.ros.org/nodelet

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/names.h>
#include <std_msgs/String.h>

namespace myNodeletClass{
    class myNodeLet : public nodelet::Nodelet{
        public:
        virtual void onInit(){
            NODELET_DEBUG("Init nodelet.");
            return;
        }
        protected:
    };
};
namespace serialPubSub{
    class serialPubSubNodelet : public nodelet::Nodelet{
        public:
        virtual void onInit(){
            // 需要使用getPrivateNodeHandle来获取nodelet的控制权
            // 直接使用ros::NodeHandle获取的是manager的控制权
            handler_ = getPrivateNodeHandle();
            pub_ = handler_.advertise<std_msgs::String>("output", 10);
            sub_ = handler_.subscribe<std_msgs::String>("input",10, &serialPubSubNodelet::subscribeCallback, this);
            ROS_INFO("%s: created.", ros::this_node::getName().c_str());
            return;
        };
        private:
        ros::NodeHandle handler_;
        ros::Publisher pub_;
        ros::Subscriber sub_ ;
        void subscribeCallback(const std_msgs::StringConstPtr& msg){
            std_msgs::String pubMsg;
            pubMsg.data = getName() + " receive: " + msg->data;
            pub_.publish(pubMsg);
            return;
        };
    };
};

#endif