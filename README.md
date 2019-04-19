# Semantic SLAM

![license](https://img.shields.io/bower/l/bootstrap.svg?color=blue) <a href="https://996.icu"><img src="https://img.shields.io/badge/link-996.icu-red.svg" alt="996.icu" /></a>

This on-going project is Semantic SLAM using ROS, ORB SLAM and PSPNet101. Will be used in Robotics for semantic understanding and navigation.

Now the visualized semantic map is achieved, where yellow represents buildings and constructions, green represents vegetation, blue represents vehicles, and red represents roads and sidewalks. You can also visualize the trajectory using Rviz. 

![semantic SLAM](https://github.com/1989Ryan/Semantic_SLAM/blob/master/semantic_map.png?raw=true)

The whole ROS communication structure of the project is shown below.

![structure](https://github.com/1989Ryan/Semantic_SLAM/blob/master/graph.png?raw=true)

The Clustering result can be visualized here. Each cluster represent an ambiguous building location.

<center>   <img src= "https://github.com/1989Ryan/README_pictures/blob/master/images/Figure_1.png?raw=true" width = "75%" height = "75%"> </center>


## Acknowledgement 

The state-of-the-art methodologies are achieved by team of [Raul Mur-Artal](https://github.com/raulmur) for [ORB_SLAM](https://github.com/raulmur/ORB_SLAM2) and team of [Hengshuang Zhao](https://github.com/hszhao) for [PSPNet](https://github.com/hszhao/PSPNet). Thanks for their great works.

The implementation of [PSPNet by keras](https://github.com/Vladkryvoruchko/PSPNet-Keras-tensorflow) is presented by [VladKry](https://github.com/Vladkryvoruchko). Thanks for their team's work.

## Prerequisite

Basic prerequisite.

* ROS kinetic
* Python 2.7
* scipy
* sklearn

To run the PSPNet in ROS, you have to install the following packages.

* Tensorflow >= 0.4.0
* Keras 2.2.2

To run the ORB_SLAM2 in ROS, you have to install the following packages.

* C++11 or C++0x Compiler
* Pangolin
* OpenCVRequired at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2.
* Eigen3 Required at least 3.1.0.
* DBoW2 and g2o (Included in Thirdparty folder)

## Project Structure

```
src/
    DBSCAN.py
    map_engine.py
Third_Part/
    ORB_SLAM/
    PSPNet_Keras_tensorflow/
test/
.gitignore
README.md
```

## RUN

You can run the script ``run.sh`` to use the system. You have to provide the rostopic ``/camera/image_raw``
```
cd Semantic_SLAM/
chmod +x run.sh
./run.sh
```

## TODO

* ~~Publish the cloud point infomation~~
* ~~Encode the cloud point and visual descriptor with semantic information~~
* ~~Clustering the cloud points into a single location point~~
* ~~Visualize the result~~
* ~~Run in the KITTI dataset~~
* ~~Use C++ for ROS node~~
* Run in simulate environment
* Improve the accuracy of Image Segmentation
* Benchmark in groundtruth
* Run in XJTU campus
* Connect all the elements into a single project
* Inference accelerate
