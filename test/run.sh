#!/bin/bash
gnome-terminal -x bash -c "roscore"
sleep 5
gnome-terminal -x bash -c "cd Third_Part/PSPNet_Keras_tensorflow/
python Semantic_Information_Publisher.py image:=/camera/image_raw"
sleep 1
gnome-terminal -x bash -c "rosrun map_generator map_generator_node /cm:=/cm /mappoint:=/mappoint /KeyPoint:=/KeyPoint"
sleep 1
gnome-terminal -x bash -c "cd Third_Part/ORB_SLAM2/
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml /camera/image_raw:=/result"
