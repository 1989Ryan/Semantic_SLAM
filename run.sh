#!/bin/bash
gnome-terminal -x bash -c "roscore"
sleep 3
gnome-terminal -x bash -c "cd Third_Part/PSPNet_Keras_tensorflow/
python Semantic_Information_Publisher.py image:=/read_image"
sleep 1
gnome-terminal -x bash -c "cd src/
python map_engine.py catorymap:=/cm KeyPoints:=/KeyPoint MapPoints:=/mappoint"
sleep 1
gnome-terminal -x bash -c "cd Third_Part/ORBSLAM2/ 
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Asus.yaml /camera/image_raw:=/result"
sleep 1
gnome-terminal -x bash -c "cd src/
python cluster.py"
