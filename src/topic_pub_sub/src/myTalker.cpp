#include <ros/ros.h>
#include <topic_pub_sub/gps.h>//auto generate gps.h
#include <topic_pub_sub/timeStamp.h>
//#include <string>
using namespace ros;
int main(int argc, char* argv[]){
    init(argc, argv, "myTalker");
    NodeHandle handler;
    topic_pub_sub::gps gpsMsg;
    topic_pub_sub::timeStamp timeSTMP;
    timeSTMP.time = "no time avail";
    gpsMsg.x = 1.0;
    gpsMsg.y = 1.0;
    gpsMsg.state = "norm";
    Publisher pubGps = handler.advertise<topic_pub_sub::gps>("gpsInfo", 1);//topic name and buffer size
    Publisher pubGpsRaw = handler.advertise<topic_pub_sub::gps>("gpsInfoRaw", 1);
    Publisher pubTimeStamp = handler.advertise<topic_pub_sub::timeStamp>("timeInfo", 1);
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