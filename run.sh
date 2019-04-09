#!/bin/bash
gnome-terminal -x bash -c "roscore"
sleep 5
gnome-terminal -x bash -c "cd Third_Part/PSPNet_Keras_tensorflow/
python Semantic_Information_Publisher.py image:=/read_image"
sleep 5
gnome-terminal -x bash -c "cd src/
python map_engine.py catorymap:=/cm KeyPoints:=/KeyPoint MapPoints:=/mappoint"
sleep 5
gnome-terminal -x bash -c " rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Asus.yaml /camera/image_raw:=/result"
