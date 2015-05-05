//
//  Created by Samantha Chen on 3/20/15.
//
//

#ifndef __Macros__
#define __Macros__

// Visualization
#define DRAW true
#define DRAW_BOXES true


// Default values for people detector
#define DEFAULT_SVM_PATH "data/trainedLinearSVMForPeopleDetectionWithHOG.yaml"
#define DEFAULT_MIN_CONFIDENCE -1.5f //-3.5f
#define DEFAULT_MIN_HEIGHT 0.5f //1.3;
#define DEFAULT_MAX_HEIGHT 2.3f //2.3;
#define DEFAULT_MIN_WIDTH 0.3f
#define DEFAULT_MAX_WIDTH 2.0f
#define DEFAULT_VOXEL_SIZE 0.15f //0.06f

// Kinect for Windows Specs
#define HORIZONTAL_FOV 57.0f
#define VERTICAL_FOV 43.0f
#define KINECT_MIN_Z 0.8f
#define KINECT_MAX_Z 4.0f

// Boundary Values
#define BOUNDARY_DIST 0.2f
#define LEFT_SIDE_BOUNDARY_DIST 0.2f
#define RIGHT_SIDE_BOUNDARY_DIST 0.2f

// To use M_PI
#define _USE_MATH_DEFINES

// Number of previous frames to check
#define NUM_FRAMES 2
// HSV color ranges
#define COLOR_MATCHING_ON true



#include <string>

struct Point {
  float x;
  float y;
  float z;
};

class Person;

class Macros{
public:
	/*
	* Converts from color from RGB to HSV 
	* Source: http://www.cs.rit.edu/~ncs/color/t_convert.html
	*/
	static void RGBtoHSV(float r, float g, float b, Point* color);
	
	/*
	* Calculates velocity using two point positions and delta time
	*/
	static void calcVelocity(float delta_time, Point* posBefore, Point* posAfter, Point* vel);

	/*
	* Checks if x,y,z coordinates is close to the boundary of Kinect's range
	*/
	static bool onKinectBoundary(float x, float y, float z);

	/*
	* Checks if point is close to the boundary of Kinect's range
	*/
	static bool onKinectBoundary(Point* p);

	// static void printInfo(Person* p);
};

#endif
