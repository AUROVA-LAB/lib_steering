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

  std::cout << "turning radius = " << turn_center_y_coordinate << std::endl;

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

bool Steering_Control::collision(double angle, double radius, const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles,
                                 const bool forward)
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

Direction Steering_Control::getBestSteering(Pose initPose, Pose finalPose)
{
  // Initialize values
  Direction bestSteering;
  bestSteering.angle = 0.0;
  bestSteering.sense = 0;
  double newDistance = 0.0;
  double minDistance = calculateMahalanobisDistance(initPose, finalPose);
  Pose nextPoseBackward = initPose;
  Pose nextPoseForward = initPose;
  Pose bestPose = initPose;
  this->actualAngle = initPose.coordinates[3];

  // Check all angles
  for (double ang = (-params.maxAngle); ang <= params.maxAngle; ang += deltaAngle)
  {
    if (true) //!collision(ang, this->length))
    {
      nextPoseForward = initPose;
      nextPoseBackward = initPose;
      for (double i = (this->deltaTime * this->velocity); i <= length; i += (this->deltaTime * this->velocity))
      {
        nextPoseForward = getNextPose(nextPoseForward, ang, 1);
        nextPoseBackward = getNextPose(nextPoseBackward, ang, -1);
        // Check the best pose forward
        newDistance = calculateMahalanobisDistance(nextPoseForward, finalPose);
        if (newDistance < minDistance)
        {
          bestSteering.angle = ang;
          bestSteering.sense = 1;
          bestPose = nextPoseForward;
          minDistance = newDistance;
        }
        // Check the best pose backward
        newDistance = calculateMahalanobisDistance(nextPoseBackward, finalPose);
        if (newDistance < minDistance)
        {
          bestSteering.angle = ang;
          bestSteering.sense = -1;
          bestPose = nextPoseBackward;
          minDistance = newDistance;
        }
      }
    }
  }

  return bestSteering;
}

Direction Steering_Control::getBestSteeringWithObstacleDetection(Pose initPose, Pose finalPose,
                                                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles)
{
  // Initialize values
  Direction bestSteering;
  bestSteering.angle = 0.0;
  bestSteering.sense = 0;
  double newDistance = 0.0;
  double currentDistance = calculateMahalanobisDistance(initPose, finalPose);
  double minDistance = 100000.0; // out ot range distance
  Pose nextPoseBackward = initPose;
  Pose nextPoseForward = initPose;
  Pose bestPose = initPose;
  this->actualAngle = initPose.coordinates[3];

  // Check all angles
  for (double ang = (-params.maxAngle); ang <= params.maxAngle; ang += deltaAngle)
  {
    // Check the best pose forward
    bool forward = true;
    if (!collision(ang, this->length, obstacles, forward))
    {
      nextPoseForward = initPose;
      for (double i = (this->deltaTime * this->velocity); i <= length; i += (this->deltaTime * this->velocity))
      {
        nextPoseForward = getNextPose(nextPoseForward, ang, 1);
        newDistance = calculateMahalanobisDistance(nextPoseForward, finalPose);
        if (newDistance < minDistance)
        {
          bestSteering.angle = ang;
          bestSteering.sense = 1;
          bestPose = nextPoseForward;
          minDistance = newDistance;
        }
      }
    }

    // Check the best pose backward
    forward = false;
    if (!collision(ang, this->length, obstacles, forward))
    {
      nextPoseBackward = initPose;
      for (double i = (this->deltaTime * this->velocity); i <= length; i += (this->deltaTime * this->velocity))
      {
        nextPoseBackward = getNextPose(nextPoseBackward, ang, -1);
        newDistance = calculateMahalanobisDistance(nextPoseBackward, finalPose);
        if (newDistance < minDistance)
        {
          bestSteering.angle = ang;
          bestSteering.sense = -1;
          bestPose = nextPoseBackward;
          minDistance = newDistance;
        }
      }
    }
  }

  //TODO: Local minima check and correction!
  return bestSteering;
}

Pose Steering_Control::getNextPose(Pose initPose, double angle, int sense)
{
  Pose nextPose = initPose;

  // Calculate with radians
  double range = velocity * deltaTime;
  double radians = angle * (M_PI / 180);
  double deltaSteering = (range / params.l) * sin(radians);

  double degrees = (deltaSteering * 180) / M_PI;

  // Saved in degrees
  nextPose.coordinates[3] += degrees;

  double radiansSteering = nextPose.coordinates[3] * (M_PI / 180);

  double deltaX = range * cos(radiansSteering) * cos(radians);
  double deltaY = range * sin(radiansSteering) * cos(radians);

  nextPose.coordinates[0] += deltaX * sense;
  nextPose.coordinates[1] += deltaY * sense;

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

