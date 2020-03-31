# lib_steering

## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), C++ template library for linear algebra

## Compilation and installation

Installation:

```shell
git clone https://github.com/AUROVA-LAB/lib_steering
cd lib_steering/
mkdir build
cd build
cmake .. 
make
sudo make install
```

## How to include it

To use this library in an other library or application, it is necessary add in the CMakeLists.txt file:

``` find_package(steering REQUIRED) ```

And Eigen's dependence is necessary:

``` 
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
```

And link the libraries to the program

``` 
TARGET_LINK_LIBRARIES(<executable name> steering) 
```

### How to use it

#### Structs:

```c++
struct Direction{
	int sense;
	double angle;
};
```
sense: Direction in which the robot moves. It has three possible values, '1' forward, '-1' backward and '0' if it does not move

angle: Turning angle

```c++
struct Pose{ 
  vector<double> coordinates; 
  vector<vector<double> > matrix; 
};
```

coordinates: Vector with coordinates x,y,z,yaw

matrix: Covariance matrix (4x4)

```c++
struct RobotParams{
	double maxAngle;
	double height;
	double width;
	double length;
};
```

maxAngle: Maximum turning angle allowed by the robot

height: Robot height 

width: Robot width

length: Robot length


#### Constructor: 

```c++	
Steering_Control(RobotParams robot, double length, double deltaTime, double deltaAngle, double velocity);
```
  
  robot: Struct with robot variables
  
  lenght: Sampling distance
  
  deltaTime: Increase in sampling time
  
  deltaAngle: Increase in sampling angle
  
  velocity; Robot velocity
  
#### Method getBestSteering:

```c++ 
	Direction getBestSteering(Pose initPose, Pose finalPose);
```	

  initPose: Pose where the robot is
  
  finalPose: Pose where the goal is
  
  return a Direction with the angle and sense

