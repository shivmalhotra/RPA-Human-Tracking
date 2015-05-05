//
//  Trajectory.cpp
//  PeopleDetector
//
//  Created by Samantha Chen on 3/3/15.
//

#include "Trajectory.h"


/**
 * Creates a new trajectory
 */
Trajectory::Trajectory() {
    positions = new std::vector<Point*>();
	velocities = new std::vector<Point*>();
	colors = new std::vector<Point*>();
}


/**
 * Disposes the trajectory, releasing all resources.
 */
Trajectory::~Trajectory() {
    delete[] positions;
	delete[] velocities;
	delete[] colors;
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