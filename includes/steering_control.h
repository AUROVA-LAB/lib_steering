#ifndef INCLUDES_STEERING_CONTROL_H_
#define INCLUDES_STEERING_CONTROL_H_
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include <bits/stdc++.h>
#include <string>

using std::vector;
using Eigen::Transpose;
using Eigen::Inverse;
using Eigen::Vector4d;
using Eigen::Matrix4d;

/**
 * Contains the information to indicate the direction of the robot
 */
struct Direction{
	int sense; // Three values, "1" forward, "-1" backward and "0" stopped
	double angle;  // Turning angle
};

/**
 * Contains the information to indicate the position of the robot
 */
struct Pose{
	vector<double> coordinates;
	vector<vector<double> > matrix;
};

/**
 * Contains the information to indicate the robot parameters
 */
struct RobotParams{
	double maxAngle; // Maximum turning angle
	double h; // Height
	double w; // Width
	double l; // Length
};

class Steering_Control {
private:

	// Struct with the robot parameters
	RobotParams st_params_;
	// Sampling distance
	double length_;
	// Robot speed
	double velocity_;
	// Increased sampling time
	double deltaTime_;
	// Sampling angle increase
	double deltaAngle_;

	/**
	 * Check if there is an object at a specific angle that could collide
	 */
	bool collision(double angle, double radius);

public:

	Steering_Control(RobotParams robot, double length, double deltaTime, double deltaAngle, double velocity);
	~Steering_Control();

	/**
	 * Returns the Mahalanobis distance of two poses
	 */
		double calculateMahalanobisDistance(Pose p1, Pose p2);
	/**
	 * Gets the next position from a starting position, an angle, and a direction
	 */
	Pose getNextPose(Pose initPose, double angle, int sense);

	/**
	 * Get the best angle and direction to get as close as possible to the final objective.
	 */
	Direction getBestSteering(Pose initPose, Pose finalPose);
};

#endif
