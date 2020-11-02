#include "../includes/SteeringControl.h"

Steering_Control::Steering_Control(RobotParams robot, double length, double deltaTime, double deltaAngle,
                                   double velocity)
{
  this->length = length;
  this->params = robot;
  this->deltaTime = deltaTime;
  this->deltaAngle = deltaAngle;
  this->velocity = velocity;
  this->actualAngle = 0;
}

Steering_Control::~Steering_Control()
{
}

void Steering_Control::filterPointsStraightLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                                const float vehicle_width, pcl::PointCloud<pcl::PointXYZI>& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aux(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough < pcl::PointXYZI > pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1 * vehicle_width / 2.0, vehicle_width / 2.0);
  pass.filter(*aux);

  pass.setInputCloud(aux);
  pass.setFilterFieldName("x");
  const float OUT_OF_RANGE = 1000.0;
  if (forward)
    pass.setFilterLimits(0.0, OUT_OF_RANGE);
  else
    pass.setFilterLimits(-1 * OUT_OF_RANGE, 0.0);

  pass.filter(output);
}

void Steering_Control::filterPointsByTurningRadius(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const bool forward,
                                                   const float steering_angle, const float wheelbase,
                                                   const float vehicle_width,
                                                   const float x_axis_distance_from_base_link_to_velodyne,
                                                   pcl::PointCloud<pcl::PointXYZI>& output)
{
  float turn_center_x_coordinate = -x_axis_distance_from_base_link_to_velodyne;
  float turn_center_y_coordinate = 0.0;
  float steering_angle_radians = steering_angle * M_PI / 180.0;

  turn_center_y_coordinate = wheelbase / tan(steering_angle_radians);

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

      if (distance < fabs(turn_center_y_coordinate) + vehicle_width / 2.0
          && distance > fabs(turn_center_y_coordinate) - vehicle_width / 2.0)
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

bool Steering_Control::collision(double angle, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles, const bool forward)
{
  //TODO: Extract as parameters! using blue values!
  const float WHEELBASE_METERS = 1.05;
  const float X_DISTANCE_FROM_BASE_LINK_TO_SENSOR = 0.550;
  const float VEHICLE_WIDTH = 0.80;
  const float SAFETY_MARGIN = 0.15;
  float safety_width = VEHICLE_WIDTH + 2.0 * SAFETY_MARGIN;

  pcl::PointCloud < pcl::PointXYZI > obstacles_filtered;

  if (fabs(angle) < 1.0) // if the steering is close to zero, the center is at infinity, so we assume straight line
  {
    this->filterPointsStraightLine(obstacles, forward, safety_width, obstacles_filtered);
  }
  else
  {
    this->filterPointsByTurningRadius(obstacles, forward, angle, WHEELBASE_METERS, safety_width,
                                      X_DISTANCE_FROM_BASE_LINK_TO_SENSOR, obstacles_filtered);
  }

  bool collision = false;
  if (obstacles_filtered.points.size() > 0)
    collision = true;

  return (collision);
}

void Steering_Control::printPosition(Pose p, string s)
{
  cout << "Position '" << s << "' : " << "(" << p.coordinates[0] << "," << p.coordinates[1] << "," << p.coordinates[2]
      << "," << p.coordinates[3] << ")" << endl;
}

Direction Steering_Control::getBestSteeringWithObstacleDetection(Pose initPose, Pose finalPose,
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
  // Initialize values
  Direction bestSteering;
  this->actualAngle = initPose.coordinates[3];
  std::cout << "actualAngle = " << actualAngle << std::endl;

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
    for (double ang = (-params.maxAngle); ang <= params.maxAngle; ang += deltaAngle)
    {
      //std::cout << "ang = " << ang << std::endl;
      // Check the best pose forward
      bool forward = true;
      //std::cout << "Checking for forward collision!" << std::endl;
      if (!collision(ang, obstacles, forward))
      {
        nextPoseForward = initPose;
        for (double distance_traveled = (this->deltaTime * this->velocity); distance_traveled <= length; distance_traveled += (this->deltaTime * this->velocity))
        {
          nextPoseForward = getNextPose(nextPoseForward, ang, distance_traveled, 1);
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
            bestSteering.angle = ang;
            bestSteering.sense = 1;
            bestPose = nextPoseForward;
            minDistance = newDistance;
          }
        }
      }
      else
      {
        //std::cout << "Forward collision detected! steering = " << ang << std::endl;
        //std::getchar();
      }

      // Check the best pose backward
      //std::cout << "Checking for backward collision!" << std::endl;
      forward = false;
      if (!collision(ang, obstacles, forward))
      {
        nextPoseBackward = initPose;
        for (double distance_traveled = (this->deltaTime * this->velocity); distance_traveled <= length; distance_traveled += (this->deltaTime * this->velocity))
        {
          nextPoseBackward = getNextPose(nextPoseBackward, ang, distance_traveled, -1);
          newDistance = calculateMahalanobisDistanceWithLocalMinima(nextPoseBackward, finalPose);
          if (newDistance < minDistance)
          {
            bestSteering.angle = ang;
            bestSteering.sense = -1;
            bestPose = nextPoseBackward;
            minDistance = newDistance;
          }
        }
      }
      else
      {
        //std::cout << "Backward collision detected! steering = " << ang << std::endl;
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
      this->local_minima_vector.push_back(initPose);
    }
  } while (local_minima);

  //std::cout << "Returning best steering!" << std::endl;
  return bestSteering;
}

Pose Steering_Control::getNextPose(Pose initPose, double angle, double distance_traveled, int sense)
{
  Pose nextPose = initPose;

  // Calculate with radians
  double steering_radians = angle * (M_PI / 180.0);

  if (fabs(steering_radians) < 0.001 * M_PI / 180.0) // to avoid infinite radius
    steering_radians = 0.001 * M_PI / 180.0;

  double r = params.l / tan(steering_radians);
  double deltaBeta = distance_traveled * (double)sense / fabs(r);
  double deltaTheta = deltaBeta;
  if(steering_radians < 0.0)
    deltaTheta = deltaTheta * -1.0;

  double deltaThetaDegrees = (deltaTheta * 180.0) / M_PI;
  // Saved in degrees
  nextPose.coordinates[3] = deltaThetaDegrees;

  double deltaX = fabs(r) * sin(deltaBeta);
  double deltaY = r * (1.0 - cos(deltaBeta));

  nextPose.coordinates[0] = deltaX;
  nextPose.coordinates[1] = deltaY;

  return nextPose;
}

double Steering_Control::calculateMahalanobisDistance(Pose p1, Pose p2)
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

double Steering_Control::calculateMahalanobisDistanceWithLocalMinima(Pose p1, Pose p2)
{
  double total_mahalanobis_distance = 0.0;

  total_mahalanobis_distance += calculateMahalanobisDistance(p1, p2);

  for (int i = 0; i < local_minima_vector.size(); i++)
  {
    double distance_to_local_minima = calculateMahalanobisDistance(p1, local_minima_vector[i]);
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

