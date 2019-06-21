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

/*TODO: Trajectory topic and message*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point.h>
#include<map_generator/mp.h>
#include<map_generator/frame.h>
#include <geometry_msgs/Pose.h>
#include <map_generator/tjy.h>
#include <nav_msgs/Path.h>


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

    void GrabImage(const map_generator::frame::ConstPtr& frame);
    void pubandsub()
    {
        pub_ = n_.advertise<sensor_msgs::PointCloud>("/Semantic_Map", 1000);
        //pub_ = n_.advertise<map_generator::mp>("/map", 1000);
        pub2 = n_.advertise<map_generator::tjy>("/trajectory", 1000);
        sub_ = n_.subscribe("/result", 1, &ImageGrabber::GrabImage, this);

    }

    ORB_SLAM2::System* mpSLAM;

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub2;
    ros::Subscriber sub_;
    sensor_msgs::PointCloud mpt;
    sensor_msgs::PointCloud kpt;
    map_generator::mp map;
    map_generator::tjy tj;
    nav_msgs::Path path;
    sensor_msgs::Image cm;
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
    s.spin();
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

//bool SortByZ(const cv::Mat &mp1, const cv::Mat &mp2)//注意：本函数的参数的类型一定要与vector中元素的类型一致  
//{   
//    return mp1.at<float>(3) < mp2.at<float>(3);  //升序排列
//}

void ImageGrabber::GrabImage(const map_generator::frame::ConstPtr& frame)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr, class_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(frame->image, frame);
        class_ptr = cv_bridge::toCvCopy(frame->category);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackMonocular(cv_ptr->image, class_ptr->image, cv_ptr->header.stamp.toSec());
    if (mpSLAM->GetTrackingState()==2)
    {
        vector<cv::Mat> mppos = mpSLAM->GetTrackedMapPointsPose();
        vector<cv::Mat> tjry = mpSLAM->GetPose();
        int i = 0;
        int num_points = mppos.size();
        path.poses.clear();
        path.header.stamp=ros::Time::now();
        path.header.frame_id="sensor_frame";
        mpt.header.stamp = ros::Time::now();
        mpt.header.frame_id = "sensor_frame";
        mpt.points.resize(num_points);
        //we'll also add an intensity channel to the cloud
        mpt.channels.resize(2);
        mpt.channels[0].name = "id";
        mpt.channels[0].values.resize(num_points);
        mpt.channels[1].name = "rgb";
        mpt.channels[1].values.resize(num_points);
        //sort(mppos.begin(), mppos.end(), SortByZ);
        geometry_msgs::PoseStamped this_pose_stamped;
        for(auto t:tjry)
        {
            this_pose_stamped.pose.position.x = t.at<float>(0);
            this_pose_stamped.pose.position.y = t.at<float>(1);
            this_pose_stamped.pose.position.z = t.at<float>(2);
            this_pose_stamped.pose.orientation.x = t.at<float>(3);
            this_pose_stamped.pose.orientation.y = t.at<float>(4);
            this_pose_stamped.pose.orientation.z = t.at<float>(5);
            this_pose_stamped.pose.orientation.w = t.at<float>(6);
            this_pose_stamped.header.stamp=path.header.stamp;
            this_pose_stamped.header.frame_id="sensor_frame";
            path.poses.push_back(this_pose_stamped);
        }
        for(auto mp: mppos)
        {
            mpt.points[i].x = mp.at<float>(0);
            mpt.points[i].y = mp.at<float>(1);
            mpt.points[i].z = mp.at<float>(2);
            mpt.channels[0].values[i] = mp.at<float>(3)*100;
            if(mpt.channels[0].values[i]>1901) mpt.channels[0].values[i]= 0;
            i++;
        }
        i = 0;
        tj.tjy = path;
        tj.gps.push_back(frame->gps);
        pub_.publish(mpt);
        pub2.publish(tj);
    }
}

