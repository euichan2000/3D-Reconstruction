// universalRobotsKinematics.h
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <stdio.h>
// #include <opencv2/highgui.hpp>
// #include <opencv2/aruco/charuco.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/calib3d/calib3d_c.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/opencv_modules.hpp>
// #include <librealsense2/rs.hpp>
#pragma once

namespace universalRobots
{

	class UR
	{


	public:

		Eigen::Matrix4f forwardKinematics(float theta[]);
		Eigen::Matrix4f calcTransformationMatrix(float alpha, float a, float d, float theta);
		float deg2rad(float degree);
		void loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ, float &X, float &Y, float &Z, float scene1[6], float scene2[6]);
		// Eigen::Matrix4f calcmarker2camera();
		// Mat createTransformationMatrix(const Mat &rotationMatrix, const Mat &translationVector);

	};

} // namespace universalRobots	