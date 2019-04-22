#!/bin/bash
gnome-terminal -x bash -c "roscore"
sleep 2
gnome-terminal -x bash -c "cd Third_Part/PSPNet_Keras_tensorflow/
python Semantic_Information_Publisher.py image:=/camera/image_raw"
sleep 1
gnome-terminal -x bash -c "cd src/
python map_engine.py"
sleep 1
gnome-terminal -x bash -c " cd Third_Part/ORB_SLAM2/
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml"
gnome-terminal -x bash -c "cd src/
python cluster.py"
