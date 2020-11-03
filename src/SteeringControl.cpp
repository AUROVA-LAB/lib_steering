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

void SteeringControl::printPosition(const Pose p, const string s)
{
  cout << "Position '" << s << "' : " << "(" << p.coordinates[0] << "," << p.coordinates[1] << "," << p.coordinates[2]
      << "," << p.coordinates[3] << ")" << endl;
}

Direction SteeringControl::getBestSteering(const Pose initPose, const Pose finalPose,
                                           const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles)
{
  /*
   std::cout << " initPose coordinates x, y, z, theta = " << initPose.coordinates[0] << ", " << initPose.coordinates[1]
   << ", " << initPose.coordinates[2] << ", " << initPose.coordinates[3] << "    variances x, y, z, theta = "
   << initPose.matrix[0][0] << ", " << initPose.matrix[1][1] << ", " << initPose.matrix[2][2] << ", "
   << initPose.matrix[3][3] << std::endl << std::endl;

   std::cout << " finalPose coordinates = " << finalPose.coordinates[0] << ", " << finalPose.coordinates[1] << ", "
   << finalPose.coordinates[2] << ", " << finalPose.coordinates[3] << "    variances x, y, z, theta = "
   << finalPose.matrix[0][0] << ", " << finalPose.matrix[1][1] << ", " << finalPose.matrix[2][2] << ", "
   << finalPose.matrix[3][3] << std::endl;
   */
  /*
   static Pose previous_final_pose;

   if(first_iteration)
   {
   previous_final_pose = finalPose;
   first_iteration = false;
   }

   double distance_between_current_and_previous_goals = calculateMahalanobisDistance(finalPose, previous_final_pose);
   if(distance_between_current_and_previous_goals > 3.0)
   {
   std::cout << "Goal change detected! clearing previous local minima information!" << std::endl;
   std::cout << "distance_between_current_and_previous_goals = " << distance_between_current_and_previous_goals << std::endl;
   local_minima_vector_.clear();
   previous_final_pose = finalPose;
   //std::getchar();
   }
   */
  // Initialize values
  Direction bestSteering;

  bool local_minima;
  do // the idea is to do this loop only once if the vehicle is not in a local minima, and make it twice otherwise
     // in the first iteration we detect the local minima, then add the initPose as local minima and recompute
     // mahalanobis distances
  {
    //std::cout << "computing current mahalanobis distance!" << std::endl;
    double currentDistance = calculateMahalanobisDistanceWithLocalMinima(initPose, finalPose);
    //std::cout << "currentDistance = " << currentDistance << std::endl;

    double minDistance = 100000000.0; // out ot range distance
    double newDistance = 0.0;

    bestSteering.angle = 0.0;
    bestSteering.sense = 0;

    Pose nextPoseBackward = initPose;
    Pose nextPoseForward = initPose;
    Pose bestPose = initPose;
    local_minima = false;
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
      if (!collision(steering_angle_deg, obstacles, forward))
      {
        nextPoseForward = initPose;
        float delta_distance = ackerman_prediction_params_.delta_time * robot_params_.max_speed_meters_per_second;
        float trajectory_length = robot_params_.max_speed_meters_per_second * ackerman_prediction_params_.temporal_horizon;

        for (float distance_traveled = delta_distance; distance_traveled <= trajectory_length; distance_traveled += delta_distance)
        {
          nextPoseForward = getNextPose(nextPoseForward, steering_angle_deg, distance_traveled, 1);
          /*
           std::cout << " nextPoseForward coordinates x, y, z, theta = " << nextPoseForward.coordinates[0] << ", "
           << nextPoseForward.coordinates[1] << ", " << nextPoseForward.coordinates[2] << ", "
           << nextPoseForward.coordinates[3] << "    variances x, y, z, theta = " << nextPoseForward.matrix[0][0]
           << ", " << nextPoseForward.matrix[1][1] << ", " << nextPoseForward.matrix[2][2] << ", "
           << nextPoseForward.matrix[3][3] << std::endl << std::endl;
           */
          newDistance = calculateMahalanobisDistanceWithLocalMinima(nextPoseForward, finalPose);

          //std::cout << "newDistance = " << newDistance << std::endl;

          //std::getchar();

          if (newDistance < minDistance)
          {
            bestSteering.angle = steering_angle_deg;
            bestSteering.sense = 1;
            bestPose = nextPoseForward;
            minDistance = newDistance;
          }
        }
      }
      else
      {
        //std::cout << "Forward collision detected! steering = " <<steering_angle_deg<< std::endl;
        //std::getchar();
      }

      // Check the best pose backward
      //std::cout << "Checking for backward collision!" << std::endl;
      forward = false;
      if (!collision(steering_angle_deg, obstacles, forward))
      {
        nextPoseBackward = initPose;
        float delta_distance = ackerman_prediction_params_.delta_time * robot_params_.max_speed_meters_per_second;
        float trajectory_length = robot_params_.max_speed_meters_per_second * ackerman_prediction_params_.temporal_horizon;

        for (float distance_traveled = delta_distance; distance_traveled <= trajectory_length; distance_traveled += delta_distance)
        {
          nextPoseBackward = getNextPose(nextPoseBackward, steering_angle_deg, distance_traveled, -1);
          newDistance = calculateMahalanobisDistanceWithLocalMinima(nextPoseBackward, finalPose);
          if (newDistance < minDistance)
          {
            bestSteering.angle = steering_angle_deg;
            bestSteering.sense = -1;
            bestPose = nextPoseBackward;
            minDistance = newDistance;
          }
        }
      }
      else
      {
        //std::cout << "Backward collision detected! steering = " <<steering_angle_deg<< std::endl;
        //std::getchar();
      }
    }

    std::cout << "bestSteering.angle = " << bestSteering.angle << std::endl;
    //std::cout << "bestSteering.sense = " << bestSteering.sense << std::endl;
    //std::cout << "minDistance = " << minDistance << std::endl;
    //std::getchar();

    //std::cout << "Checking local minima!" << std::endl;
    if (minDistance >= currentDistance)
    {
      std::cout << "Local minima found!" << std::endl;
      //getchar();
      local_minima = true;
      this->local_minima_vector_.push_back(initPose);
    }
  } while (local_minima);

  //std::cout << "Returning best steering!" << std::endl;
  return bestSteering;
}

Pose SteeringControl::getNextPose(const Pose initPose, const float steering_angle_deg, const float distance_traveled,
                                  const int sense)
{
  Pose nextPose = initPose;

  // Calculate with radians
  float steering_radians = steering_angle_deg * (M_PI / 180.0);

  if (fabs(steering_radians) < 0.001 * M_PI / 180.0) // to avoid infinite radius
    steering_radians = 0.001 * M_PI / 180.0;

  float r = robot_params_.wheelbase / tan(steering_radians);
  float deltaBeta = distance_traveled * (float)sense / fabs(r);
  float deltaTheta = deltaBeta;
  if (steering_radians < 0.0)
    deltaTheta = deltaTheta * -1.0;

  float deltaThetaDegrees = (deltaTheta * 180.0) / M_PI;
  // Saved in degrees
  nextPose.coordinates[3] = deltaThetaDegrees;

  float deltaX = fabs(r) * sin(deltaBeta);
  float deltaY = r * (1.0 - cos(deltaBeta));

  nextPose.coordinates[0] = deltaX;
  nextPose.coordinates[1] = deltaY;

  return nextPose;
}

double SteeringControl::calculateMahalanobisDistance(const Pose p1, const Pose p2)
{
  Vector4d vectorX;
  Vector4d vectorG;
  Vector4d vectorZ;
  Matrix4d matrixP;
  Matrix4d matrixQ;
  Matrix4d matrixZ;

  for (unsigned int i = 0; i < p2.coordinates.size(); i++)
  {
    vectorX(i) = p2.coordinates[i];
  }
  for (unsigned int i = 0; i < p1.coordinates.size(); i++)
  {
    vectorG(i) = p1.coordinates[i];

  }
  for (unsigned int i = 0; i < p2.matrix.size(); i++)
  {
    for (unsigned int j = 0; j < p2.matrix[i].size(); j++)
    {
      matrixP(i, j) = p2.matrix[i][j];
      matrixQ(i, j) = p1.matrix[i][j];
    }
  }

  vectorZ = vectorG - vectorX;
  matrixZ = matrixP * matrixQ * matrixP.transpose();
  double aux = (vectorZ.transpose() * matrixZ.inverse() * vectorZ).value();
  double mahalanobisDistance = sqrt(aux);

  return mahalanobisDistance;
}

double SteeringControl::calculateMahalanobisDistanceWithLocalMinima(const Pose p1, const Pose p2)
{
  double total_mahalanobis_distance = 0.0;

  total_mahalanobis_distance += calculateMahalanobisDistance(p1, p2);

  for (int i = 0; i < local_minima_vector_.size(); i++)
  {
    double distance_to_local_minima = calculateMahalanobisDistance(p1, local_minima_vector_[i]);
    //TODO: Extract as parameter!
    const double MAHAB_DISTANCE_THRESHOLD_TO_IGNORE_LOCAL_MINIMA = 3.0;
    if (distance_to_local_minima < MAHAB_DISTANCE_THRESHOLD_TO_IGNORE_LOCAL_MINIMA)
    {
      double penalty = (MAHAB_DISTANCE_THRESHOLD_TO_IGNORE_LOCAL_MINIMA / (0.001 + distance_to_local_minima)) - 1.0;
      if (penalty > 0)
        total_mahalanobis_distance += penalty;
    }
  }

  return (total_mahalanobis_distance);
}

