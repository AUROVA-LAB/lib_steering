#ifndef INCLUDES_STEERINGCONTROL_H_
#define INCLUDES_STEERINGCONTROL_H_
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <bits/stdc++.h>
#include <string>

using namespace std;
using namespace Eigen;

struct Direction{
	int sense;
	double angle;
};

struct Pose{
	vector<double> coordinates;
	vector<vector<double> > matrix;
};

struct RobotParams{
	double maxAngle;
	double h;
	double w;
	double l;
};

class Steering_Control {
private:
	RobotParams params;
	double length;
	double velocity;
	double actualAngle;
	double deltaTime;
	double deltaAngle;

	bool collision(double angle, double radius);

public:
	Steering_Control(RobotParams robot, double length, double deltaTime, double deltaAngle, double velocity);
	~Steering_Control();

	void printPosition(Pose p, string s);
	double calculateMahalanobisDistance(Pose p1, Pose p2);
	Pose getNextPose(Pose initPose, double angle, int sense);
	Direction getBestSteering(Pose initPose, Pose finalPose);
	Direction getBestSteeringWithObstacleDetection(Pose initPose, Pose finalPose, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles);
};

#endif
