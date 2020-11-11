#include "../includes/SteeringControl.h"

SteeringControl::SteeringControl(const RobotParams robot_params,
                                 const AckermannPredictionParams ackerman_prediction_params,
                                 const AckermannControlParams ackermann_control_params,
                                 const CollisionAvoidanceParams collision_avoidance_params)
{
  this->robot_params_ = robot_params;
  this->ackerman_prediction_params_ = ackerman_prediction_params;
  this->ackermann_control_params_ = ackermann_control_params;
  this->collision_avoidance_params_ = collision_avoidance_params;

  this->first_iteration_ = true;
}

SteeringControl::~SteeringControl()
{
}

void SteeringControl::filterPointsStraightLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                               const float safety_width, pcl::PointCloud<pcl::PointXYZI>& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aux(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough < pcl::PointXYZI > pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1 * safety_width / 2.0, safety_width / 2.0);
  pass.filter(*aux);

  pass.setInputCloud(aux);
  pass.setFilterFieldName("x");
  const float OUT_OF_RANGE = 1000.0;
  if (forward)
    pass.setFilterLimits(robot_params_.x_distance_from_velodyne_to_front, OUT_OF_RANGE);
  else
    pass.setFilterLimits(-1 * OUT_OF_RANGE, -1.0 * robot_params_.x_distance_from_velodyne_to_back);

  pass.filter(output);
}

void SteeringControl::filterPointsByTurningRadius(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                                  const float steering_angle_deg, const float safety_width,
                                                  pcl::PointCloud<pcl::PointXYZI>& output)
{
  float turn_center_x_coordinate = -1.0 * robot_params_.x_distance_from_velodyne_to_base_link;
  float turn_center_y_coordinate = 0.0;
  float steering_angle_radians = steering_angle_deg * M_PI / 180.0;

  turn_center_y_coordinate = robot_params_.wheelbase / tan(steering_angle_radians);

  //std::cout << "turning radius = " << turn_center_y_coordinate << std::endl;

  float x = 0.0;
  float y = 0.0;
  float distance;

  for (size_t i = 0; i < input->points.size(); ++i)
  {
    x = input->points[i].x;
    if ((forward && x > 0.0) || (!forward && x <= 0.0))
    {
      y = input->points[i].y;

      distance = sqrt(
          (x - turn_center_x_coordinate) * (x - turn_center_x_coordinate)
              + (y - turn_center_y_coordinate) * (y - turn_center_y_coordinate));

      if (distance < fabs(turn_center_y_coordinate) + safety_width / 2.0
          && distance > fabs(turn_center_y_coordinate) - safety_width / 2.0)
      {
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = input->points[i].z;
        output.points.push_back(point);
      }
    }
  }
}

bool SteeringControl::collision(const float steering_angle_deg, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles,
                                const bool forward)
{
  float safety_width = robot_params_.width + 2.0 * collision_avoidance_params_.safety_lateral_margin;

  pcl::PointCloud < pcl::PointXYZI > obstacles_filtered;

  if (fabs(steering_angle_deg) < 0.001) // if the steering is close to zero, the center is at infinity, so we assume straight line
  {
    this->filterPointsStraightLine(obstacles, forward, safety_width, obstacles_filtered);
  }
  else
  {
    this->filterPointsByTurningRadius(obstacles, forward, steering_angle_deg, safety_width, obstacles_filtered);
  }

  bool collision = false;
  if (obstacles_filtered.points.size() > 0)
    collision = true;

  return (collision);
}

float SteeringControl::findMaxRecommendedSpeed(const float steering_angle_deg,
                                               const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles, const bool forward)
{
  //std::cout << "findMaxRecommendedSpeed" << std::endl;

  float safety_width = robot_params_.width + 2.0 * collision_avoidance_params_.safety_lateral_margin;

  pcl::PointCloud < pcl::PointXYZI > obstacles_filtered;

  if (fabs(steering_angle_deg) < 0.001) // if the steering is close to zero, the center is at infinity, so we assume straight line
  {
    this->filterPointsStraightLine(obstacles, forward, safety_width, obstacles_filtered);
  }
  else
  {
    this->filterPointsByTurningRadius(obstacles, forward, steering_angle_deg, safety_width, obstacles_filtered);
  }

  static const float OUT_OF_RANGE = 10000.0;

  float x = 0.0;
  float y = 0.0;
  float distance;
  float min_front_distance = OUT_OF_RANGE;
  float min_back_distance = OUT_OF_RANGE;

  for (size_t i = 0; i < obstacles_filtered.points.size(); ++i)
  {
    x = obstacles_filtered.points[i].x;
    y = obstacles_filtered.points[i].y;

    distance = sqrt(x * x + y * y);
    if (x < 0.0 && distance < min_back_distance)
    {
      min_back_distance = distance;
    }
    if (x > 0.0 && distance < min_front_distance)
    {
      min_front_distance = distance;
    }
  }

  float max_recommended_speed = 0.0;

  if (forward)
  {
    float safety_length = robot_params_.x_distance_from_velodyne_to_front
        + collision_avoidance_params_.safety_longitudinal_margin;

    max_recommended_speed = (min_front_distance - safety_length)
        / collision_avoidance_params_.time_to_reach_min_allowed_distance;
  }
  else
  {
    float safety_length = robot_params_.x_distance_from_velodyne_to_back
        + collision_avoidance_params_.safety_longitudinal_margin;

    max_recommended_speed = (min_back_distance - safety_length)
        / collision_avoidance_params_.time_to_reach_min_allowed_distance;
  }

  if (max_recommended_speed > ackermann_control_params_.max_speed_meters_per_second)
    max_recommended_speed = ackermann_control_params_.max_speed_meters_per_second;

  if (max_recommended_speed < 0.0)
  {
    max_recommended_speed = 0.0;
    std::cout << "Warning in SteeringControl::findMaxRecommendedSpeed, negative speed!" << std::endl;
  }
  return (max_recommended_speed);
}

void SteeringControl::printPosition(const Pose p, const string s)
{
  cout << "Position '" << s << "' : " << "(" << p.coordinates[0] << "," << p.coordinates[1] << "," << p.coordinates[2]
      << "," << p.coordinates[3] << ")" << endl;
}

SteeringAction SteeringControl::getBestSteeringAction(const Pose initPose, const Pose finalPose,
                                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles)
{

  std::cout << " initPose coordinates x, y, z, theta = " << initPose.coordinates[0] << ", " << initPose.coordinates[1]
      << ", " << initPose.coordinates[2] << ", " << initPose.coordinates[3] << std::endl << "Covariance matrix: "
      << std::endl << initPose.matrix[0][0] << ", " << initPose.matrix[0][1] << ", " << initPose.matrix[0][2] << ", "
      << initPose.matrix[0][3] << std::endl << initPose.matrix[1][0] << ", " << initPose.matrix[1][1] << ", "
      << initPose.matrix[1][2] << ", " << initPose.matrix[1][3] << std::endl << initPose.matrix[2][0] << ", "
      << initPose.matrix[2][1] << ", " << initPose.matrix[2][2] << ", " << initPose.matrix[2][3] << std::endl
      << initPose.matrix[3][0] << ", " << initPose.matrix[3][1] << ", " << initPose.matrix[3][2] << ", "
      << initPose.matrix[3][3] << std::endl;

  std::cout << " finalPose coordinates = " << finalPose.coordinates[0] << ", " << finalPose.coordinates[1] << ", "
      << finalPose.coordinates[2] << ", " << finalPose.coordinates[3] << std::endl << "Covariance matrix: " << std::endl
      << finalPose.matrix[0][0] << ", " << finalPose.matrix[0][1] << ", " << finalPose.matrix[0][2] << ", "
      << finalPose.matrix[0][3] << std::endl << finalPose.matrix[1][0] << ", " << finalPose.matrix[1][1] << ", "
      << finalPose.matrix[1][2] << ", " << finalPose.matrix[1][3] << std::endl << finalPose.matrix[2][0] << ", "
      << finalPose.matrix[2][1] << ", " << finalPose.matrix[2][2] << ", " << finalPose.matrix[2][3] << std::endl
      << finalPose.matrix[3][0] << ", " << finalPose.matrix[3][1] << ", " << finalPose.matrix[3][2] << ", "
      << finalPose.matrix[3][3] << std::endl;

  ///////////// Detecting goal change and cleaning past local minima information if the goal changes
  static Pose previous_final_pose;

  if (first_iteration_)
  {
    previous_final_pose = finalPose;
    first_iteration_ = false;
  }

  std::cout << " previous_final_pose coordinates = " << previous_final_pose.coordinates[0] << ", "
      << previous_final_pose.coordinates[1] << ", " << previous_final_pose.coordinates[2] << ", "
      << previous_final_pose.coordinates[3] << std::endl << "Covariance matrix: " << std::endl
      << previous_final_pose.matrix[0][0] << ", " << previous_final_pose.matrix[0][1] << ", "
      << previous_final_pose.matrix[0][2] << ", " << previous_final_pose.matrix[0][3] << std::endl
      << previous_final_pose.matrix[1][0] << ", " << previous_final_pose.matrix[1][1] << ", "
      << previous_final_pose.matrix[1][2] << ", " << previous_final_pose.matrix[1][3] << std::endl
      << previous_final_pose.matrix[2][0] << ", " << previous_final_pose.matrix[2][1] << ", "
      << previous_final_pose.matrix[2][2] << ", " << previous_final_pose.matrix[2][3] << std::endl
      << previous_final_pose.matrix[3][0] << ", " << previous_final_pose.matrix[3][1] << ", "
      << previous_final_pose.matrix[3][2] << ", " << previous_final_pose.matrix[3][3] << std::endl;

  double distance_between_current_and_previous_goals = calculateBhattacharyyaDistance(finalPose, previous_final_pose);
  std::cout << "distance_between_current_and_previous_goals = " << distance_between_current_and_previous_goals
      << std::endl;
  if (distance_between_current_and_previous_goals > 0.001)
  {
    std::cout << "Goal change detected! clearing previous local minima information!" << std::endl;
    std::cout << "distance_between_current_and_previous_goals = " << distance_between_current_and_previous_goals
        << std::endl;

    std::cout << "Number of local minima before cleaning = " << local_minima_vector_.size() << std::endl;
    local_minima_vector_.clear();
    previous_final_pose = finalPose;
    std::cout << "Number of local minima AFTER cleaning = " << local_minima_vector_.size() << std::endl;
    //std::getchar();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Initialize values
  SteeringAction bestAction;

  int do_while_iteration_number = 0;
  bool local_minima;
  do // the idea is to do this loop only once if the vehicle is not in a local minima, and make it twice otherwise
     // in the first iteration we detect the local minima, then add the initPose as local minima and recompute
     // mahalanobis distances
  {
    do_while_iteration_number++;
    std::cout << "do while iteration number = " << do_while_iteration_number << std::endl;
    //std::cout << "computing current mahalanobis distance!" << std::endl;
    double currentDistance = calculateBhattacharyyaDistanceWithLocalMinima(initPose, finalPose);
    std::cout << "currentDistance = " << currentDistance << std::endl;
    std::cout << "currentEuclidean distance = "
        << sqrt(
            (initPose.coordinates[0] - finalPose.coordinates[0]) * (initPose.coordinates[0] - finalPose.coordinates[0])
                + (initPose.coordinates[1] - finalPose.coordinates[1])
                    * (initPose.coordinates[1] - finalPose.coordinates[1])) << std::endl;

    double minDistance = 100000000.0; // out ot range distance
    double newDistance = 0.0;

    bestAction.angle = 0.0;
    bestAction.sense = 0; //Sense of zero stops the vehicle
    bestAction.speed = 0.0;

    Pose nextPoseBackward = initPose;
    Pose nextPoseForward = initPose;
    Pose bestPose = initPose;

    local_minima = false;
    bool there_is_at_least_one_valid_action = false;
    // Check all angles
    //std::cout << "Checking all angles!" << std::endl;
    for (float steering_angle_deg = -1.0 * robot_params_.abs_max_steering_angle_deg;
        steering_angle_deg <= robot_params_.abs_max_steering_angle_deg; steering_angle_deg +=
            ackerman_prediction_params_.delta_steering)
    {
      //std::cout << "steering_angle_deg = " << steering_angle_deg << std::endl;
      // Check the best pose forward
      bool forward = true;
      //std::cout << "Checking for forward collision!" << std::endl;
      float recommendedSpeed = findMaxRecommendedSpeed(steering_angle_deg, obstacles, forward);
      if (recommendedSpeed > robot_params_.min_speed_meters_per_second)
      {
        there_is_at_least_one_valid_action = true;
        //std::cout << "Forward: predicting costs at steering_angle_deg = " << steering_angle_deg << std::endl;
        nextPoseForward = initPose;
        float delta_distance = ackerman_prediction_params_.delta_time * robot_params_.max_speed_meters_per_second;
        float trajectory_length = robot_params_.max_speed_meters_per_second
            * ackerman_prediction_params_.temporal_horizon;

        for (float distance_traveled = delta_distance; distance_traveled <= trajectory_length; distance_traveled +=
            delta_distance)
        {
          nextPoseForward = getNextPose(initPose, steering_angle_deg, distance_traveled, 1);
          /*
           std::cout << " nextPoseForward coordinates x, y, z, theta = " << nextPoseForward.coordinates[0] << ", "
           << nextPoseForward.coordinates[1] << ", " << nextPoseForward.coordinates[2] << ", "
           << nextPoseForward.coordinates[3] << "    variances x, y, z, theta = " << nextPoseForward.matrix[0][0]
           << ", " << nextPoseForward.matrix[1][1] << ", " << nextPoseForward.matrix[2][2] << ", "
           << nextPoseForward.matrix[3][3] << std::endl << std::endl;
           */
          newDistance = calculateBhattacharyyaDistanceWithLocalMinima(nextPoseForward, finalPose);

          //std::cout << "distance_traveled = " << distance_traveled << "    newMahalanobisDistance = " << newDistance << std::endl;

          //std::getchar();

          if (newDistance < minDistance)
          {
            bestAction.angle = steering_angle_deg;
            bestAction.sense = 1;
            bestAction.speed = recommendedSpeed;
            bestPose = nextPoseForward;
            minDistance = newDistance;
          }
        }
      }
      else
      {
        //std::cout << "Forward collision detected! steering = " << steering_angle_deg << std::endl;
        //std::getchar();
      }

      //std::getchar();

      // Check the best pose backward
      //std::cout << "Checking for backward collision!" << std::endl;
      forward = false;
      recommendedSpeed = findMaxRecommendedSpeed(steering_angle_deg, obstacles, forward);
      if (recommendedSpeed > robot_params_.min_speed_meters_per_second)
      {
        there_is_at_least_one_valid_action = true;
        //std::cout << "Backward: predicting costs at steering_angle_deg = " << steering_angle_deg << std::endl;
        nextPoseBackward = initPose;
        float delta_distance = ackerman_prediction_params_.delta_time * robot_params_.max_speed_meters_per_second;
        float trajectory_length = robot_params_.max_speed_meters_per_second
            * ackerman_prediction_params_.temporal_horizon;

        for (float distance_traveled = delta_distance; distance_traveled <= trajectory_length; distance_traveled +=
            delta_distance)
        {
          nextPoseBackward = getNextPose(initPose, steering_angle_deg, distance_traveled, -1);
          newDistance = calculateBhattacharyyaDistanceWithLocalMinima(nextPoseBackward, finalPose);

          //std::cout << "distance_traveled = " << distance_traveled << "    newMahalanobisDistance = " << newDistance << std::endl;

          if (newDistance < minDistance)
          {
            bestAction.angle = steering_angle_deg;
            bestAction.sense = -1;
            bestAction.speed = recommendedSpeed;
            bestPose = nextPoseBackward;
            minDistance = newDistance;
          }
        }
      }
      else
      {
        //std::cout << "Backward collision detected! steering = " << steering_angle_deg << std::endl;
        //std::getchar();
      }
    }

    std::cout << "minDistance = " << minDistance << std::endl;
    std::cout << "bestAction.angle = " << bestAction.angle << std::endl;
    std::cout << "bestAction.sense = " << bestAction.sense << std::endl;
    std::cout << "bestAction.speed = " << bestAction.speed << std::endl;
    //std::getchar();

    bool forward = true;
    if (bestAction.sense == -1)
      forward = false;

    std::cout << "forward = " << forward << std::endl;

    //std::cout << "Checking local minima!" << std::endl;
    if (minDistance >= currentDistance && there_is_at_least_one_valid_action)
    {
      /*
       std::cout << "Local minima found!" << std::endl;
       //getchar();
       local_minima = true;
       Pose local_minima_pose = initPose;

       Eigen::Matrix2d control_matrix;
       control_matrix << 9.0, 0.0, 0.0, 1.0;

       double yaw_rad = local_minima_pose.coordinates[3] * M_PI / 180.0;

       Eigen::Matrix2d rot;
       rot << cos(yaw_rad), sin(yaw_rad), -1.0 * sin(yaw_rad), cos(yaw_rad);

       control_matrix = rot * control_matrix;

       local_minima_pose.matrix[0][0] += control_matrix(0, 0);
       local_minima_pose.matrix[0][1] += control_matrix(0, 1);
       local_minima_pose.matrix[1][0] += control_matrix(1, 0);
       local_minima_pose.matrix[1][1] += control_matrix(1, 1);

       this->local_minima_vector_.push_back(local_minima_pose);
       std::cout << "Total number of local minima = " << local_minima_vector_.size() << std::endl;
       */
    }
  } while (local_minima);

  std::cout << "Total number of local minima = " << local_minima_vector_.size() << std::endl;
  //std::cout << "Returning best steering!" << std::endl;
  return bestAction;
}

Pose SteeringControl::getNextPose(const Pose initPose, const float steering_angle_deg, const float distance_traveled,
                                  const int sense)
{
  Pose next_pose_in_baselink_frame = initPose;

  // Calculate with radians
  float steering_radians = steering_angle_deg * (M_PI / 180.0);

  if (fabs(steering_radians) < 0.001 * M_PI / 180.0) // to avoid infinite radius
    steering_radians = 0.001 * M_PI / 180.0;

  float r = robot_params_.wheelbase / tan(steering_radians);
  float deltaBeta = distance_traveled * (float)sense / fabs(r);
  float deltaTheta = deltaBeta;
  if (steering_radians < 0.0)
    deltaTheta = deltaTheta * -1.0;

  float deltaX = fabs(r) * sin(deltaBeta);
  float deltaY = r * (1.0 - cos(deltaBeta));

  Vector2f robot_pos_inc_in_baselink_frame;
  robot_pos_inc_in_baselink_frame << deltaX, deltaY;

  float robot_init_yaw_rad = initPose.coordinates[3] * M_PI / 180.0;
  Matrix2f robot_init_rot;
  robot_init_rot << cos(robot_init_yaw_rad), -1.0 * sin(robot_init_yaw_rad), sin(robot_init_yaw_rad), cos(
      robot_init_yaw_rad);

  Vector2f robot_initial_trans;
  robot_initial_trans << initPose.coordinates[0], initPose.coordinates[1];

  Vector2f robot_final_trans;
  robot_final_trans = robot_initial_trans + robot_init_rot * robot_pos_inc_in_baselink_frame;

  next_pose_in_baselink_frame.coordinates[0] = robot_final_trans(0);
  next_pose_in_baselink_frame.coordinates[1] = robot_final_trans(1);

  float deltaThetaDegrees = (deltaTheta * 180.0) / M_PI;
  // Saved in degrees
  next_pose_in_baselink_frame.coordinates[3] += deltaThetaDegrees;

  return next_pose_in_baselink_frame;
}

double SteeringControl::calculateMahalanobisDistance(const Pose p1, const Pose p2)
{
  Vector2d x;
  Vector2d g;
  Vector2d z;
  Matrix2d P;
  Matrix2d Q;
  Matrix2d Z;

  g(0) = p1.coordinates[0];
  g(1) = p1.coordinates[1];

  Q(0, 0) = p1.matrix[0][0];
  Q(0, 1) = p1.matrix[0][1];
  Q(1, 0) = p1.matrix[1][0];
  Q(1, 1) = p1.matrix[1][1];

  x(0) = p2.coordinates[0];
  x(1) = p2.coordinates[1];

  P(0, 0) = p2.matrix[0][0];
  P(0, 1) = p2.matrix[0][1];
  P(1, 0) = p2.matrix[1][0];
  P(1, 1) = p2.matrix[1][1];

  z = g - x;
  Z = P * Q * P.transpose();

  double aux = (z.transpose() * Z.inverse() * z).value();
  double mahalanobisDistance = sqrt(aux);

  if (std::isnan(mahalanobisDistance))
  {
    std::cout << "g = " << g << std::endl;
    std::cout << "Q = " << Q << std::endl << std::endl;

    std::cout << "x = " << x << std::endl;
    std::cout << "P = " << P << std::endl << std::endl;

    std::cout << "z = " << z << std::endl;
    std::cout << "Z = " << Z << std::endl << std::endl;

    std::cout << "squared mahalanobis distance = " << aux << std::endl;
    std::cout << "mahalanobis distance = " << mahalanobisDistance << std::endl << std::endl;

    assert(false && "Error computing Mahalanobis distance!");
  }

  return mahalanobisDistance;
}

double SteeringControl::calculateMahalanobisDistanceWithLocalMinima(const Pose p1, const Pose p2)
{
  double total_mahalanobis_distance = calculateMahalanobisDistance(p1, p2);

  for (int i = 0; i < local_minima_vector_.size(); i++)
  {
    //std::cout << "Current Mahalanobis distance to goal = " << total_mahalanobis_distance << std::endl;

    double distance_to_local_minima = calculateMahalanobisDistance(p1, local_minima_vector_[i]);
    //std::cout << "Current Mahalanobis distance to local minima number " << i << " = " << distance_to_local_minima
    //    << std::endl;
    if (distance_to_local_minima < ackermann_control_params_.mahalanobis_distance_threshold_to_ignore_local_minima)
    {
      double penalty = (ackermann_control_params_.mahalanobis_distance_threshold_to_ignore_local_minima
          / (0.001 + distance_to_local_minima)) - 1.0;
      //std::cout << "Penalty for local minima number " << i << " = " << penalty << std::endl;

      if (penalty > 0)
        total_mahalanobis_distance += penalty;

      //std::cout << "Current Mahalanobis distance to goal after local minima " << i << " = "
      //    << total_mahalanobis_distance << std::endl;
    }
  }

  //if (local_minima_vector_.size() > 0)
  //  std::cout << "Final Mahalanobis distance to goal = " << total_mahalanobis_distance << std::endl;
  return (total_mahalanobis_distance);
}

double SteeringControl::calculateBhattacharyyaDistance(const Pose p1, const Pose p2)
{
  Vector2d x;
  Vector2d g;
  Vector2d z;
  Matrix2d P;
  Matrix2d Q;
  Matrix2d Z;

  g(0) = p1.coordinates[0];
  g(1) = p1.coordinates[1];

  Q(0, 0) = p1.matrix[0][0];
  Q(0, 1) = p1.matrix[0][1];
  Q(1, 0) = p1.matrix[1][0];
  Q(1, 1) = p1.matrix[1][1];

  x(0) = p2.coordinates[0];
  x(1) = p2.coordinates[1];

  P(0, 0) = p2.matrix[0][0];
  P(0, 1) = p2.matrix[0][1];
  P(1, 0) = p2.matrix[1][0];
  P(1, 1) = p2.matrix[1][1];

  z = g - x;
  Z = (P + Q) / 2.0;

  double first_term = (z.transpose() * Z.inverse() * z);
  double second_term = log(Z.determinant() / (sqrt(Q.determinant() * P.determinant()))) / 2.0;

  double bhattacharyya_distance = (first_term / 8.0) + second_term;

  //double bhattacharyya_distance = first_term;
  //std::cout << "first  bhattacharyya_distance term = " << z.transpose() * Z.inverse() * z << std::endl;
  //std::cout << "second bhattacharyya_distance term = " <<  (log(Z.determinant() / (sqrt(Q.determinant() * P.determinant()))) / 2.0) << std::endl;

  if (std::isnan(bhattacharyya_distance) || bhattacharyya_distance < 0.0)
  {
    std::cout << "g = " << g << std::endl;
    std::cout << "Q = " << Q << std::endl << std::endl;

    std::cout << "x = " << x << std::endl;
    std::cout << "P = " << P << std::endl << std::endl;

    std::cout << "z = " << z << std::endl;
    std::cout << "Z = " << Z << std::endl;
    std::cout << "Z.inverse = " << Z.inverse() << std::endl << std::endl;

    std::cout << "bhattacharyya_distance = " << bhattacharyya_distance << std::endl << std::endl;

    assert(false && "Error computing bhattacharyya_distance!");
  }

  return bhattacharyya_distance;
}

double SteeringControl::calculateBhattacharyyaDistanceWithLocalMinima(const Pose p1, const Pose p2)
{
  double total_bhattacharyya_distance = calculateBhattacharyyaDistance(p1, p2);

  for (int i = 0; i < local_minima_vector_.size(); i++)
  {
    //std::cout << "Current total_bhattacharyya_distance to goal = " << total_bhattacharyya_distance << std::endl;

    double distance_to_local_minima = calculateBhattacharyyaDistance(p1, local_minima_vector_[i]);
    //std::cout << "Current bhattacharyya_distance to local minima number " << i << " = " << distance_to_local_minima
    //    << std::endl;
    if (distance_to_local_minima < ackermann_control_params_.mahalanobis_distance_threshold_to_ignore_local_minima)
    {
      double penalty = (ackermann_control_params_.mahalanobis_distance_threshold_to_ignore_local_minima
          / (0.001 + distance_to_local_minima)) - 1.0;
      //std::cout << "Penalty for local minima number " << i << " = " << penalty << std::endl;

      if (penalty > 0)
        total_bhattacharyya_distance += penalty;

      //std::cout << "Current bhattacharyya_distance to goal after local minima " << i << " = "
      //    << total_bhattacharyya_distance << std::endl;
    }
  }

  //if (local_minima_vector_.size() > 0)
  //  std::cout << "Final bhattacharyya_distance to goal = " << total_bhattacharyya_distance << std::endl;
  return (total_bhattacharyya_distance);
}

