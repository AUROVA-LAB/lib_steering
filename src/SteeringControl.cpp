#include "../includes/SteeringControl.h"

Steering_Control::Steering_Control(RobotParams robot, double length, double deltaTime, double deltaAngle, double velocity){
	this->length = length;
	this->params = robot;
	this->deltaTime = deltaTime;
	this->deltaAngle = deltaAngle;
	this->velocity = velocity;
	this->actualAngle=0;
}

Steering_Control::~Steering_Control() {
}

bool Steering_Control::collision(double angle, double radius){
	return false;
}

void Steering_Control::printPosition(Pose p, string s){
	cout << "Position '" << s << "' : " << "(" << p.coordinates[0] << "," << p.coordinates[1] << "," << p.coordinates[2] << "," << p.coordinates[3] << ")" << endl;
}

Direction Steering_Control::getBestSteering(Pose initPose, Pose finalPose){

	// Initialize values
	Direction bestSteering;
	bestSteering.angle = 0;
	bestSteering.sense = 0;
	double newDistance=0;
	double minDistance = calculateMahalanobisDistance(initPose, finalPose);
	Pose nextPoseBackward=initPose;
	Pose nextPoseForward=initPose;
	Pose bestPose=initPose;
	this->actualAngle=initPose.coordinates[3];

	// Check all angles
	for(double ang=(-params.maxAngle); ang<=params.maxAngle; ang+= deltaAngle){
		if(!collision(ang,this->length)){
			nextPoseForward = initPose;
			nextPoseBackward = initPose;
			for(double i=(this->deltaTime*this->velocity); i<=length; i+=(this->deltaTime*this->velocity)){
				nextPoseForward = getNextPose(nextPoseForward,ang,1);
				nextPoseBackward = getNextPose(nextPoseBackward,ang,-1);
				// Check the best pose forward
				newDistance = calculateMahalanobisDistance(nextPoseForward,finalPose);
				if( newDistance< minDistance){
					bestSteering.angle=ang;
					bestSteering.sense = 1;
					bestPose = nextPoseForward;
					minDistance = newDistance;
				}
				// Check the best pose backward
				newDistance = calculateMahalanobisDistance(nextPoseBackward,finalPose);
				if( newDistance< minDistance){
					bestSteering.angle=ang;
					bestSteering.sense = -1;
					bestPose = nextPoseBackward;
					minDistance = newDistance;
				}
			}
		}
	}

	return bestSteering;
}

Pose Steering_Control::getNextPose(Pose initPose, double angle, int sense){
	Pose nextPose=initPose;

	// Calculate with radians
	double range = velocity * deltaTime;
	double radians=angle*(M_PI/180);
	double deltaSteering = (range/params.l)*sin(radians);

	double degrees = (deltaSteering *180)/M_PI;

	// Saved in degrees
	nextPose.coordinates[3] += degrees;

	double radiansSteering = nextPose.coordinates[3]*(M_PI/180);

	double deltaX = range * cos(radiansSteering) * cos(radians);
	double deltaY = range * sin(radiansSteering) * cos(radians);

	nextPose.coordinates[0] += deltaX*sense;
	nextPose.coordinates[1] += deltaY*sense;

	return nextPose;
}


double Steering_Control::calculateMahalanobisDistance(Pose p1, Pose p2){

	Vector4d vectorX;
	Vector4d vectorG;
	Vector4d vectorZ;
	Matrix4d matrixP;
	Matrix4d matrixQ;
	Matrix4d matrixZ;

	for (unsigned int i = 0; i < p2.coordinates.size(); i++)
	{
		vectorX(i)= p2.coordinates[i];
	}
	for (unsigned int i = 0; i < p1.coordinates.size(); i++)
	{
		vectorG(i)= p1.coordinates[i];

	}
	for (unsigned int i = 0; i < p2.matrix.size(); i++)
	{
		for (unsigned int j = 0; j < p2.matrix[i].size(); j++){
			matrixP(i,j)= p2.matrix[i][j];
			matrixQ(i,j)= p1.matrix[i][j];
		}
	}

	vectorZ = vectorG-vectorX;
	matrixZ = matrixP*matrixQ*matrixP.transpose();
	double aux = (vectorZ.transpose()*matrixZ.inverse()*vectorZ).value();
	double mahalanobisDistance = sqrt(aux);

	return mahalanobisDistance;
}





