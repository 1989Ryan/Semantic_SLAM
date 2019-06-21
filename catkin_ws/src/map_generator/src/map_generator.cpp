#include<iostream>
#include<algorithm>
#include<fstream>
#include<sensor_msgs/PointCloud.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/Point.h>
#include<map_generator/mp.h>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include<queue>

#define QUEUESIZE = 30


using namespace std;

class Map_Generator
{
public:
    Map_Generator(void){};

    void pubandsub()
    {
        sub_ = n_.subscribe("/map", 1000, &Map_Generator::callback, this);
        pub_ = n_.advertise<sensor_msgs::PointCloud>("/Semantic_Map", 1);
        ROS_INFO("node inited");
        smp.channels.resize(1);
        smp.channels[0].name = "category";
        mpt.channels.resize(1);
        kpt.channels.resize(1);
    }
/*
    void encodelabel()z
    {
        //smp.channels[0].values[]
    }

    void bayesupdate()
    {
      //todo
    }*/

	
    void callback( const map_generator::mp::ConstPtr& msg)
    {
        int num_points = msg->mpt.channels[0].values.size();
        int num_keys = msg->kpt.channels[0].values.size();
        //queueing the messages
        if(kpqueue.size()>40)
            kpqueue.pop();
        kpqueue.push(msg->kpt);
        if(mpqueue.size()>40)
            mpqueue.pop();
        mpqueue.push(msg->mpt);
        if(framequeue.size()>40)
            framequeue.pop();
        framequeue.push(msg->currentframe);
        //cout<<"good2"<<endl;
        //queueing(msg->mpt, mpqueue);
        //queueing(msg->currentframe, framequeue);
        if(framequeue.size()<=40)
            return ;
        cv_bridge::CvImageConstPtr cv_ptr;
        //cout<<"good"<<endl;
        //currentframe = framequeue.front();
        try
        {
            cv_ptr = cv_bridge::toCvCopy(framequeue.front());
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //cout<<"good3"<<endl;
        int num_points0 = mpqueue.front().channels[0].values.size();
        //cout<<"good4"<<endl;
        int num_keys0 = kpqueue.front().channels[0].values.size();
        //cout<<"good5"<<endl;
        smp.points.resize(num_points);
        mpt.points.resize(num_points);
        kpt.points.resize(num_keys0);
        smp.points = msg->mpt.points;
        //cout<<"good6"<<endl;
        //we'll also add an intensity channel to the cloud
        smp.channels[0].values.resize(num_points);
        //cout<<"good7"<<endl;
        mpt.channels[0].values.resize(num_points);
        //cout<<"good8"<<endl;
        kpt.channels[0].values.resize(num_keys0);
        //cout<<"good9"<<endl;
        vector <float> id;
        id.resize(num_points);
        //cout<<"good8"<<endl;
        mpt =  msg->mpt;
        kpt = kpqueue.front();
        id = mpt.channels[0].values;
        //cout<<"good9"<<endl;
        int counter = 0;
        for(int k = 0; k< num_keys0; k++)
        {
            geometry_msgs::Point32 kp = kpt.points[k];
            cv::Mat img = cv_ptr->image;
            //int category = int(cv_ptr->image.at<uchar>(int(kp.y), int(kp.x)));
            int category = (int)(*(img.data+img.step[0]*int(kp.y)+img.step[1]*int(kp.x)));
            vector <float>::iterator iElement = find(id.begin(), id.end(), kp.z);
            if( iElement != id.end() )
            {
                int i = distance(id.begin(),iElement);
                smp.channels[0].values[i] = category * 100;
            }
        }
        smp.header.stamp = mpt.header.stamp;
        smp.header.frame_id = mpt.header.frame_id;
        pub_.publish(smp);
        ROS_INFO("semantic map published");
    }

protected:
    sensor_msgs::PointCloud smp;

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    sensor_msgs::ImageConstPtr currentframe;
    sensor_msgs::PointCloud mpt;
    sensor_msgs::PointCloud kpt;
    int count;
    queue<sensor_msgs::PointCloud> mpqueue;
    queue<sensor_msgs::PointCloud> kpqueue;
    queue<sensor_msgs::Image> framequeue;
    //vector<Eigen::Matrix<double,19,1>> probs;
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
