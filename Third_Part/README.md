# ThirdPart Implementation Using ROS

We try to use ROS to establish the connection between PSPNet101 and ORB SLAM. This project is currently on-going.

## Prerequisite

To run the PSPNet in ROS, you have to install the following packages.

* python 2.7
* ROS Kinetic
* Tensorflow >= 0.4.0
* Keras 2.2.2

To run the ORB_SLAM2 in ROS, you have to install the following packages.

* C++11 or C++0x Compiler
* Pangolin
* OpenCVRequired at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2.
* Eigen3 Required at least 3.1.0.
* DBoW2 and g2o (Included in Thirdparty folder)
* ROS Kinetic

## Semantic Image Publisher

Run ``python Semantic_Information_Publisher.py image:={the image topic you provide}`` to publish the images and categories for each pixels.

## Cloud Publish

Run ``ros_mono.cc`` to publish the cloud point.

You can visualize the result with Rviz.

![](https://github.com/1989Ryan/Semantic_SLAM/blob/master/Third_Part/Screenshot%20from%202019-03-29%2013-20-20.png?raw=true)

