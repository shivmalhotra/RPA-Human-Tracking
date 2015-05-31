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
  times = new std::vector<float>();
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
  delete[] times;
	delete color;
}

void Trajectory::addPosition(float x, float y, float z) {	
	Point* p = new Point(); 
	p->x = x; 
	p->y = y; 
	p->z = z; 
	positions->push_back(p);	
}
  
void Trajectory::addVelocity(float x, float y, float z) {	
	Point* p = new Point(); 
	p->x = x; 
	p->y = y; 
	p->z = z; 
	velocities->push_back(p);    
}

void Trajectory::addColor(float h, float s, float v) {	
	Point* p = new Point(); 
	p->x = h; 
	p->y = s; 
	p->z = v; 
	colors->push_back(p);    
}


