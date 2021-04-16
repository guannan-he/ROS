#include <ros/ros.h>
#include <topicPubSub/gps.h>//auto generate gps.h
#include <topicPubSub/timeStamp.h>
//#include <string>
using namespace ros;
int main(int argc, char* argv[]){
    init(argc, argv, "myTalker");
    NodeHandle handler;
    topicPubSub::gps gpsMsg;
    topicPubSub::timeStamp timeSTMP;
    timeSTMP.time = "no time avail";
    gpsMsg.x = 1.0;
    gpsMsg.y = 1.0;
    gpsMsg.state = "norm";
    Publisher pubGps = handler.advertise<topicPubSub::gps>("gpsInfo", 1);//topic name and buffer size
    Publisher pubGpsRaw = handler.advertise<topicPubSub::gps>("gpsInfoRaw", 1);
    Publisher pubTimeStamp = handler.advertise<topicPubSub::timeStamp>("timeInfo", 1);
    Rate loopRate(1.0);//init freq at 1Hz
    while (ok()){
        gpsMsg.x *= 1.03;
        gpsMsg.y *= 1.03;
        ROS_INFO("Talker gps info: x cord: %2.3f, y cord: %2.3f", gpsMsg.x, gpsMsg.y);
        ROS_INFO("Talker time info: %s", timeSTMP.time.c_str());
        pubGps.publish(gpsMsg);
        pubGpsRaw.publish(gpsMsg);
        pubTimeStamp.publish(timeSTMP);
        loopRate.sleep();
    }
    return 0;
}