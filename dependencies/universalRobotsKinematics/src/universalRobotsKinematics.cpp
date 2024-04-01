// universalRobotsKinematics.cpp

#include "universalRobotsKinematics.h"
#include <random>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
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

void universalRobots::UR::loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ, float &X, float &Y, float &Z, float scene1[6], float scene2[6])
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
}

/*
Mat createTransformationMatrix(const Mat &rotationMatrix, const Mat &translationVector)
{
	// 4x4 크기의 변환 행렬을 생성합니다.
	Mat transformationMatrix = Mat::eye(4, 4, CV_64F);

	// 회전 행렬 A를 변환 행렬의 상단 왼쪽 3x3 부분에 복사합니다.
	Mat rotationPart = transformationMatrix(Rect(0, 0, 3, 3));
	Mat transpose_rotationmatrix;
	transpose(rotationMatrix, transpose_rotationmatrix);
	transpose_rotationmatrix.copyTo(rotationPart);

	// 이동 벡터 B를 변환 행렬의 우측 열에 복사합니다.
	Mat translationPart = transformationMatrix.col(3).rowRange(0, 3);
	Mat translationmatrix = -1 * transpose_rotationmatrix * translationVector;
	translationmatrix.copyTo(translationPart);

	return transformationMatrix;
}

Eigen::Matrix4f calcmarker2cam()
{
	Mat image;
	char buf[256];
	image = imread(images[i]);

	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
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

			matrix_target2cam = createTransformationMatrix(rmatrix_image, tvec_image);
		}
	}
}
*/
