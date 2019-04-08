/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>


#include<mutex>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/MapPoint.h"
#include"../../../include/Map.h"

using namespace std;
using namespace ORB_SLAM2;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void pubandsub()
    {
        pub_ = n_.advertise<sensor_msgs::PointCloud>("/mappoint", 1000);
        pub_2 = n_.advertise<sensor_msgs::PointCloud>("/KeyPoint",1);
        sub_ = n_.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, this);
    }

    ORB_SLAM2::System* mpSLAM;

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub_2;
    ros::Subscriber sub_;
    sensor_msgs::PointCloud mpt;
    sensor_msgs::PointCloud kpt;
    int count;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ImageGrabber igb(&SLAM);
    igb.pubandsub();
    ros::MultiThreadedSpinner s(2);
    ros::spin();
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveMap("MapPointandKeyFrame.bin");
    SLAM.SavePoint("MapPoint.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    if (mpSLAM->GetTrackingState()==2)
    {
        vector<cv::Mat> mppos = mpSLAM->GetTrackedMapPointsPose();
        vector<cv::Point3f> key = mpSLAM -> GetGoodKeyPoints();
        int i = 0;
        int num_points = mppos.size();
        mpt.header.stamp = ros::Time::now();
        mpt.header.frame_id = "sensor_frame";
        mpt.points.resize(num_points);
        //we'll also add an intensity channel to the cloud
        mpt.channels.resize(1);
        mpt.channels[0].name = "rgb";
        mpt.channels[0].values.resize(num_points);
        for(auto mp: mppos)
        {
            mpt.points[i].x = mp.at<float>(0);
            mpt.points[i].y = mp.at<float>(1);
            mpt.points[i].z = mp.at<float>(2);
            mpt.channels[0].values[i] = mp.at<float>(3);
            i++;
        }
        i = 0;
        kpt.points.resize(key.size());
        kpt.header.stamp = ros::Time::now();
        kpt.header.frame_id = "keypoint_frame";
        //we'll also add an intensity channel to the cloud
        kpt.channels.resize(1);
        kpt.channels[0].name = "rgb";
        kpt.channels[0].values.resize(key.size());
        for(auto k: key)
        {
            kpt.points[i].x = k.x;
            kpt.points[i].y = k.y;
            kpt.points[i].z = k.z;
            //cout<<kp.kp[i].x <<" "<<kp.kp[i].y<<endl;
            i ++;
        }
        i = 0;
        pub_.publish(mpt);
        pub_2.publish(kpt);
    }
}
