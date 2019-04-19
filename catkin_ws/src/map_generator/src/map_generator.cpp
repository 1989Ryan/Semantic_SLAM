#include<iostream>
#include<algorithm>
#include<fstream>
#include<sensor_msgs/PointCloud.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/Point.h>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<opencv2/core/core.hpp>

using namespace std;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::PointCloud> slamsyncpolicy;

class Map_Generator
{
public:
    Map_Generator(void){};

    void pubandsub()
    {
        sub_ = n_.subscribe("/cm", 1000, &Map_Generator::callback, this);
        sub2_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(n_, "/mappoint", 1000);
        sub3_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(n_, "/KeyPoint", 1000);
        sync_ = new message_filters::Synchronizer<slamsyncpolicy>(slamsyncpolicy(100), *sub2_, *sub3_);
        sync_ -> registerCallback(boost::bind(&Map_Generator::callback2, this, _1, _2));
        pub_ = n_.advertise<sensor_msgs::PointCloud>("/smp", 1);
        ROS_INFO("node inited");
        smp.channels.resize(1);
        smp.channels[0].name = "category";
    }
    void callback(const sensor_msgs::ImageConstPtr& img)
    {
        currentframe = img;
    }

    void callback2( const sensor_msgs::PointCloudConstPtr& mpt, const sensor_msgs::PointCloudConstPtr& kpt)
    {
        
        int num_points = mpt->channels[0].values.size();
        int num_keys = kpt->channels[0].values.size();
        smp.points.resize(num_points);
        smp.points = mpt->points;
        //we'll also add an intensity channel to the cloud
        smp.channels[0].values.resize(num_points);
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(currentframe);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        vector <float> id;
        id.resize(num_points);
        id = mpt->channels[0].values;
        for(int k = 0; k< num_keys; k++)
        {
            geometry_msgs::Point32 kp = kpt->points[k];
            int category = cv_ptr->image.ptr<uchar>(int(kp.y))[int(kp.x)];
            vector <float>::iterator iElement = find(id.begin(), id.end(), float(kp.z));
            if( iElement != id.end() )
            {
                int i = distance(id.begin(),iElement);
                smp.channels[0].values[i] = category * 100;
            }
        }
        smp.header.stamp = mpt->header.stamp;
        smp.header.frame_id = mpt->header.frame_id;
        pub_.publish(smp);
        ROS_INFO("semantic map published");
    }

protected:
    sensor_msgs::PointCloud smp;

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud>* sub2_;
    message_filters::Subscriber<sensor_msgs::PointCloud>* sub3_;
    message_filters::Synchronizer<slamsyncpolicy>* sync_;
    sensor_msgs::ImageConstPtr currentframe;
    int count;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"Map_Generator");
    ros::start();
    Map_Generator mgtr;
    mgtr.pubandsub();
    ros::MultiThreadedSpinner s(3);
    s.spin();
    ros::shutdown();
    return 0;
}
