// universalRobotsKinematics.h
#pragma once
#include <random>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <librealsense2/rs.hpp>
#include <yaml-cpp/yaml.h>

namespace universalRobots
{

	class UR
	{

	public:
		Eigen::Matrix4f forwardKinematics(float theta[]);
		Eigen::Matrix4f calcTransformationMatrix(float alpha, float a, float d, float theta);
		float deg2rad(float degree);
		void loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ, float &X, float &Y, float &Z, float scene[][6], int &n, float minrange[4], float maxrange[4],float &statisticfilterparam,float &downsampleparam);
		Eigen::Matrix4f createTransformationMatrix(const cv::Mat &rotationMatrix, const cv::Mat &translationVector);
		Eigen::Matrix4f calccam2marker(const std::string &filename);
	};
} // namespace universalRobots