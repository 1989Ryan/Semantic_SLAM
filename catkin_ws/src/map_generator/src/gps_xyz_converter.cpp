#include<iostream>
#include<sensor_msgs/NavSatFix.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<map_generator/tjy.h>
#include<Eigen/Dense>
#include<queue>
#include<ros/ros.h>

#define QUEUESIZE = 10

using namespace std;

class gps_xyz_converter
{
public:
    gps_xyz_converter(void){};
    
    void pubandsub()
    {
        sub = n.subscribe("/trajectory", 1000, &gps_xyz_converter::callback, this);
    }
    void callback(const map_generator::tjy::ConstPtr& msg)
    {
        gpsq.push(msg->gps.front());
        if(gpsq.size()>10)
            gpsq.pop();
        poseq.push(msg->tjy.poses.front());
        if(poseq.size()>10)
            poseq.pop();

        

        
    }

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    queue<sensor_msgs::NavSatFix> gpsq;
    queue<geometry_msgs::PoseStamped> poseq;
};
