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

struct SteeringAction
{
  int sense;
  float angle;
  float max_recommended_speed_meters_per_second;
};

struct Pose
{
  vector<double> coordinates;
  vector<vector<double> > matrix;
};

struct RobotParams
{
  float x_distance_from_velodyne_to_base_link;
  float x_distance_from_velodyne_to_front;
  float x_distance_from_velodyne_to_back;
  float abs_max_steering_angle_deg;
  float wheelbase;
  float width;
  float length;
  float height;
  float ground_z_coordinate_in_sensor_frame_origin;
  float max_speed_meters_per_second;
  float min_speed_meters_per_second;
};

struct AckermannPredictionParams
{
  float temporal_horizon;
  float delta_time;
  float delta_steering;
};

struct AckermannControlParams
{
  float max_speed_meters_per_second;
  float min_speed_meters_per_second;
  float max_delta_speed;
  float final_goal_approximation_radius;
  double mahalanobis_distance_threshold_to_ignore_local_minima;
};

struct CollisionAvoidanceParams
{
  float safety_lateral_margin;
  float safety_above_margin;
  float safety_longitudinal_margin;
  float min_obstacle_height;
  float time_to_reach_min_allowed_distance;
};

class SteeringControl
{
private:
  RobotParams robot_params_;
  AckermannPredictionParams ackerman_prediction_params_;
  AckermannControlParams ackermann_control_params_;
  CollisionAvoidanceParams collision_avoidance_params_;

  bool first_iteration_;

  std::vector<Pose> local_minima_vector_;

  void filterPointsStraightLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                float safety_width, pcl::PointCloud<pcl::PointXYZI>& output);

  void filterPointsByTurningRadius(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                   const float steering_angle_deg, const float safety_width,
                                   pcl::PointCloud<pcl::PointXYZI>& output);

  bool collision(const float steering_angle_deg, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles,
                 const bool forward);

  float findMaxRecommendedSpeed(const float steering_angle_deg, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles,
                                const bool forward);

  double calculateMahalanobisDistanceWithLocalMinima(const Pose p1, const Pose p2);

public:
  SteeringControl(const RobotParams robot_params, const AckermannPredictionParams ackerman_prediction_params,
                  const AckermannControlParams ackermann_control_params,
                  const CollisionAvoidanceParams collision_avoidance_params);
  ~SteeringControl();

  void printPosition(const Pose p, const string s);
  double calculateMahalanobisDistance(const Pose p1, const Pose p2);
  Pose getNextPose(const Pose initPose, const float steering_angle_deg, const float distance_traveled, const int sense);
  SteeringAction getBestSteeringAction(const Pose initPose, const Pose finalPose,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles);
};

#endif
