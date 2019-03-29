# ThirdPart Implementation Using ROS

We try to use ROS to establish the connection between PSPNet101 and ORB SLAM. This project is currently on-going.

## Prerequisite

To run the PSPNet in ROS, you have to install the following packages.

* python 2.7
* ROS Kinetic
* Tensorflow >= 0.4.0
* Keras 2.2.2

## Semantic Image Publisher

Run ``python Semantic_Information_Publisher.py image:={the image topic you provide}`` to publish the segmented images.

## Cloud Publish

Run ``ros_mono.cc`` to publish the cloud point.

You can visualize the result with Rviz.

![](https://github.com/1989Ryan/Semantic_SLAM/blob/master/Third_Part/Screenshot%20from%202019-03-29%2013-20-20.png?raw=true)
