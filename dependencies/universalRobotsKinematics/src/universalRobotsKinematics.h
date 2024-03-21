// universalRobotsKinematics.h
#include <Eigen/Dense>

#pragma once

namespace universalRobots
{

	class UR
	{


	public:

		Eigen::Matrix4f forwardKinematics(float theta[]);
		Eigen::Matrix4f calcTransformationMatrix(float alpha, float a, float d, float theta);
		float deg2rad(float degree);
	};

} // namespace universalRobots	