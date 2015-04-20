//  HospitalFalls
//  Macros.cpp
//
//  Created by Samantha Chen on 3/22/2015

#include "Person.h"
#include "Macros.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <fstream>

// Source: http://www.cs.rit.edu/~ncs/color/t_convert.html
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
// if s == 0, then h = -1 (undefined)
void Macros::RGBtoHSV( float r, float g, float b, Point* color)
{
	float min, max, delta;
	min = std::min( r, std::min(g, b) );
	max = std::max( r, std::max(g, b) );
	color->z = max;				// v
	delta = max - min;
	
	if( max != 0 )
		color->y = delta / max;		// s
	else {
		// r = g = b = 0		// s = 0, v is undefined
		color->y = 0;
		color->x = -1;
		return;
	}

	if(delta != 0){ // if delta == 0 then r,g,b all have same value so h,s = 0
	
	if( r == max ) 
		color->x = ( g - b ) / delta;		// between yellow & magenta
	else if( g == max )
		color->x = 2 + ( b - r ) / delta;	// between cyan & yellow
	else
		color->x = 4 + ( r - g ) / delta;	// between magenta & cyan
	color->x *= 60;				// degrees
	if( color->x < 0 )
		color->x += 360;
	
	} else {
		color->x = 0;
		color->y = 0;
	}
}

void Macros::calcVelocity (float delta_time, Point* posBefore, Point* posAfter, Point* vel) 
{
	vel->x = ((posAfter->x - posBefore->x) + 0.5) / delta_time;
	vel->y = ((posAfter->y - posBefore->y) + 0.5) / delta_time;
	vel->z = ((posAfter->z - posBefore->z) + 0.5) / delta_time;
}

bool Macros::onKinectBoundary (float x, float y, float z) {
	// top and bottom boundaries	
	if (z < KINECT_MIN_Z + BOUNDARY_DIST || z > KINECT_MAX_Z - BOUNDARY_DIST) {
		return true;
	}
	// left side boundary
	float x0 = x;
	float y0 = z;
	float b = -1.0f * tan(HORIZONTAL_FOV/2.0f * M_PI/180.0f);
	float a = 1.0f;
	float c = 0.0f;
	float d = fabs(a * x0 + b * y0 + c) / sqrt(a*a + b*b);
	//std::cout << d << std::endl;		

	if (d < SIDE_BOUNDARY_DIST) {
		return true;
	}
	
	//right side boundary
	b = tan(HORIZONTAL_FOV/2.0f * M_PI/180.0f);
	d = fabs(a * x0 + b * y0 + c) / sqrt(a*a + b*b);
	//std::cout << d << std::endl;		
	//std::cout << " " << std::endl;
	
	if (d < SIDE_BOUNDARY_DIST) {
		return true;
	}
	else {
		return false;
	}
}

bool Macros::onKinectBoundary(Point* p) {
	return Macros::onKinectBoundary(p->x,p->y,p->z);
}

// void Macros::printInfo(Person* p){
// 	ofstream myfile;
// 	myfile.open(filename, std::ios_base::app);
	
// 	myfile << "Person: " << p->getID() 
// 	<< " x: " << p->getTrajectory()->getPosition()->x 
// 	<< " y: " << p->getTrajectory()->getPosition()->y 
// 	<< " z: " << p->getTrajectory()->getPosition()->z 
// 	<<std::endl;
	
// 	myfile
// 	<< " h: " << p->getColor()->x 
// 	<< " s: " << p->getColor()->y 
// 	<< " v: " << p->getColor()->z 
// 	<<std::endl;
	
// 	myfile.close();
// }
