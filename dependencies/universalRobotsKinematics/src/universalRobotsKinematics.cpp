// universalRobotsKinematics.cpp

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

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <librealsense2/rs.hpp>
#include <yaml-cpp/yaml.h>
#include "universalRobotsKinematics.h"

using namespace cv;
using namespace std;
using namespace aruco;

Eigen::Matrix4f universalRobots::UR::forwardKinematics(float theta[])
{

	float alpha[6] = {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0.0};
	float a[6] = {0.0, -0.612, -0.5723, 0.0, 0.0, 0.0};
	float d[6] = {0.1273, 0.0, 0.0, 0.163941, 0.1157, 0.0922};

	for (int i = 0; i < 6; ++i)
	{
		theta[i] = deg2rad(theta[i]);
	}

	Eigen::Matrix4f T01 = calcTransformationMatrix(alpha[0], a[0], d[0], theta[0]);
	Eigen::Matrix4f T12 = calcTransformationMatrix(alpha[1], a[1], d[1], theta[1]);
	Eigen::Matrix4f T23 = calcTransformationMatrix(alpha[2], a[2], d[2], theta[2]);
	Eigen::Matrix4f T34 = calcTransformationMatrix(alpha[3], a[3], d[3], theta[3]);
	Eigen::Matrix4f T45 = calcTransformationMatrix(alpha[4], a[4], d[4], theta[4]);
	Eigen::Matrix4f T56 = calcTransformationMatrix(alpha[5], a[5], d[5], theta[5]);

	return T01 * T12 * T23 * T34 * T45 * T56;
}

Eigen::Matrix4f universalRobots::UR::calcTransformationMatrix(float alpha, float a, float d, float theta)
{
	Eigen::Matrix4f individualTransformationMatrix;
	individualTransformationMatrix << cos(theta), (-sin(theta) * cos(alpha)), (sin(theta) * sin(alpha)), (a * cos(theta)),
		sin(theta), (cos(theta) * cos(alpha)), (-cos(theta) * sin(alpha)), (sin(theta) * a),
		0, sin(alpha), cos(alpha), d,
		0, 0, 0, 1;
	return individualTransformationMatrix;
}

float universalRobots::UR::deg2rad(float degree)
{
	return degree * M_PI / 180.0;
}

void universalRobots::UR::loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ, float &X, float &Y, float &Z, float scene1[6], float scene2[6], float scene3[6], float scene4[6], float minrange[4],float maxrange[4])
{
	// YAML 파일을 읽어들일 ifstream 객체를 생성합니다.
	std::ifstream fin(filename);

	// YAML을 파싱하고 데이터를 저장할 YAML::Node 객체를 생성합니다.
	YAML::Node doc = YAML::Load(fin);

	// hand_eye_calibration 정보 불러오기
	thetaX = doc["hand_eye_calibration"][0].as<float>() * M_PI / 180;
	thetaY = doc["hand_eye_calibration"][1].as<float>() * M_PI / 180;
	thetaZ = doc["hand_eye_calibration"][2].as<float>() * M_PI / 180;
	X = doc["hand_eye_calibration"][3].as<float>();
	Y = doc["hand_eye_calibration"][4].as<float>();
	Z = doc["hand_eye_calibration"][5].as<float>();

	// scene1 배열 불러오기
	int i = 0;
	for (const auto &element : doc["scene1"])
	{
		scene1[i++] = element.as<float>();
	}

	// scene2 배열 불러오기
	int j = 0;
	for (const auto &element : doc["scene2"])
	{
		scene2[j++] = element.as<float>();
	}
	// scene3 배열 불러오기
	int k = 0;
	for (const auto &element : doc["scene3"])
	{
		scene3[k++] = element.as<float>();
	}
	// scene4 배열 불러오기
	int l = 0;
	for (const auto &element : doc["scene4"])
	{
		scene4[l++] = element.as<float>();
	}
	int m = 0;
	for (const auto &element : doc["minrange"])
	{
		minrange[m++] = element.as<float>();
	}
	int n = 0;
	for (const auto &element : doc["maxrange"])
	{
		maxrange[n++] = element.as<float>();
	}

}

Eigen::Matrix4f universalRobots::UR::createTransformationMatrix(const Mat &rotationMatrix, const Mat &translationVector)
{
	Eigen::Matrix3f rotationEigen;
	Eigen::Vector3f translationEigen;

	// OpenCV Mat 객체를 Eigen 행렬로 변환합니다.
	cv::cv2eigen(rotationMatrix, rotationEigen);
	cv::cv2eigen(translationVector, translationEigen);

	Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();

	// 회전 행렬을 변환 행렬의 상단 왼쪽 3x3 부분에 복사합니다.
	transformationMatrix.block<3, 3>(0, 0) = rotationEigen;

	// 이동 벡터를 변환 행렬의 우측 열에 복사합니다.
	transformationMatrix.block<3, 1>(0, 3) = translationEigen;

	return transformationMatrix;
}

Eigen::Matrix4f universalRobots::UR::calccam2marker(const std::string &filename)
{

	Mat cameraMatrix = (Mat_<double>(3, 3) << 898.454561024777, 0, 651.946687160551,
						0, 899.092148378796, 382.332250962326,
						0, 0, 1);
	Mat distCoeffs = (Mat_<double>(1, 5) << 0.008861249213184505, 0.3494378753891278, -0.002162889885546831, 0.006092371833764963, -1.084121168410527);
	Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_250);
	Ptr<CharucoBoard> board = CharucoBoard::create(7, 5, 0.035, 0.005, dictionary);
	Ptr<DetectorParameters> params = DetectorParameters::create();
	cv::Mat rvec_image, tvec_image, rmatrix_image;
	Eigen::Matrix4f matrix_cam2target;
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners;
	universalRobots::UR robot;

	// if at least one marker detected

	Mat image;
	char buf[256];
	image = imread(filename);
	detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
	// if at least one marker detected
	if (markerIds.size() > 0)
	{
		drawDetectedMarkers(image, markerCorners, markerIds);
		vector<Point2f> charucoCorners;
		vector<int> charucoIds;
		interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
		// if at least one charuco corner detected
		if (charucoIds.size() > 0)
		{
			Scalar color = Scalar(255, 0, 0);
			drawDetectedCornersCharuco(image, charucoCorners, charucoIds, color);

			bool valid = estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec_image, tvec_image);

			Rodrigues(rvec_image, rmatrix_image); // convert rotation vector to rotation matrix

			matrix_cam2target = robot.createTransformationMatrix(rmatrix_image, tvec_image);

			// if charuco pose is valid
			if (valid)
			{
				drawFrameAxes(image, cameraMatrix, distCoeffs, rvec_image, tvec_image, 0.1f);
			}
		}
	}
	// sprintf(buf, "../image/charuco_with_axis.jpg");
	// imwrite(buf, image);
	// imshow("Image", image);
	// waitKey(0);
	return matrix_cam2target;

	// Return an identity matrix if transformation could not be estimated
	return Eigen::Matrix4f::Identity();
}
