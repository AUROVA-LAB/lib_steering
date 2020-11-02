#ifndef INCLUDES_STEERINGCONTROL_H_
#define INCLUDES_STEERINGCONTROL_H_
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <bits/stdc++.h>
#include <string>

using namespace std;
using namespace Eigen;

struct Direction
{
  int sense;
  double angle;
};

struct Pose
{
  vector<double> coordinates;
  vector<vector<double> > matrix;
};

struct RobotParams
{
  double maxAngle;
  double h;
  double w;
  double l;
};

class Steering_Control
{
private:
  RobotParams params;
  double length;
  double velocity;
  double actualAngle;
  double deltaTime;
  double deltaAngle;

  std::vector<Pose> local_minima_vector;

  void filterPointsStraightLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                float vehicle_width, pcl::PointCloud<pcl::PointXYZI>& output);

  void filterPointsByTurningRadius(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                   const float steering_angle, const float wheelbase, const float vehicle_width,
                                   const float x_axis_distance_from_base_link_to_velodyne,
                                   pcl::PointCloud<pcl::PointXYZI>& output);

  bool collision(double angle, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles, const bool forward);

  double calculateMahalanobisDistanceWithLocalMinima(Pose p1, Pose p2);

public:
  Steering_Control(RobotParams robot, double length, double deltaTime, double deltaAngle, double velocity);
  ~Steering_Control();

  void printPosition(Pose p, string s);
  double calculateMahalanobisDistance(Pose p1, Pose p2);
  Pose getNextPose(Pose initPose, double angle, double distance_traveled, int sense);
  Direction getBestSteeringWithObstacleDetection(Pose initPose, Pose finalPose,
                                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles);
};

#endif
