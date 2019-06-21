/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);//时间，维度，经度，海拔，精确的位姿，回调函数，导入gps信息
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);//时间，里程计的路径（平移部分），旋转部分（四元数），回调函数，导入里程计信息
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);//获取融合后的位姿信息
	nav_msgs::Path global_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);//将GPS信息转成当前坐标系的坐标
	void optimize();//融合算法的实现
	void updateGlobalPath();//更新全局位姿

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;//保存vio的位姿，double存的是时间t，第二个参数存的是位姿向量
	map<double, vector<double>> globalPoseMap;//保存优化后的全局位姿，数据格式同上
	map<double, vector<double>> GPSPositionMap;//gps数据，数据格式同上
	bool initGPS;
	bool newGPS;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

};