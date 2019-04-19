# ThirdPart Implementation Using ROS

We try to use ROS to establish the connection between PSPNet101 and ORB SLAM. This project is currently on-going.

## Prerequisite

To run the PSPNet in ROS, you have to install the following packages.

* python 2.7
* ROS Kinetic
* Tensorflow >= 0.4.0
* Keras 2.2.2

You have to use the weight files for PSPNet, which can be download [here](https://pan.baidu.com/s/1hg1MF5Tysae-s1Hcv0F-lA). Key: j6cm.

After you download the weight file, you have to put it at the folder ``PSPNet_Keras_tensorflow/weights/npy/`` to make sure that it could be used currectly. Make sure that the default version of python is python 2.7.

To run the ORB_SLAM2 in ROS, you have to install the following packages.

* C++11 or C++0x Compiler
* Pangolin
* OpenCVRequired at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2.
* Eigen3 Required at least 3.1.0.
* DBoW2 and g2o (Included in Thirdparty folder)
* ROS Kinetic

Please run the file ``build.sh`` and ``build_ros.sh`` to compile the files. Then you have to add ``export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:{Your own path}/Semantic_SLAM/Third_Part/ORB_SLAM2/Examples/ROS`` to ``.bashrc``.

## Semantic Image Publisher

Run ``python Semantic_Information_Publisher.py image:={the image topic you provide}`` to publish the images and categories for each pixels.

## Cloud Publish

Run ``ros_mono.cc`` to publish the cloud point.

You can visualize the result with Rviz.

