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
