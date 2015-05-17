//
//  Trajectory.cpp
//  PeopleDetector
//
//  Created by Samantha Chen on 3/3/15.
//

#include "Trajectory.h"
int Trajectory::nextID = 0;

/**
 * Creates a new trajectory
 */
Trajectory::Trajectory() {
    positions = new std::vector<Point*>();
	velocities = new std::vector<Point*>();
	colors = new std::vector<Point*>();
	color = new Point();
	id = Trajectory::nextID++;
}


/**
 * Disposes the trajectory, releasing all resources.
 */
Trajectory::~Trajectory() {
    delete[] positions;
	delete[] velocities;
	delete[] colors;
	delete color;
}

/**
* Adds a position to the trajectory.
*
* @param value The position.
*/
void Trajectory::addPosition(float x, float y, float z) {	
	Point* p = new Point(); 
	p->x = x; 
	p->y = y; 
	p->z = z; 
	positions->push_back(p);	
}
    
/**
* Adds a velocity to the trajectory.
*
* @return The velocity.
*/
void Trajectory::addVelocity(float x, float y, float z) {	
	Point* p = new Point(); 
	p->x = x; 
	p->y = y; 
	p->z = z; 
	velocities->push_back(p);    
}

void Trajectory::addColor(Point* p) {	
	colors->push_back(p);  
}

void Trajectory::setHSVColor(float h, float s, float v) {
	color->x = h;
	color->y = s;
	color->z = v; 
}

void Trajectory::setRGBColor(float r, float g, float b) {
	Macros::RGBtoHSV(r,g,b,color);
}