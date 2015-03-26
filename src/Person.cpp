//
//  Person.cpp
//  PeopleDetector
//
//  Created by Samantha Chen on 3/3/15.
//

#include "Person.h"
int Person::nextID = 0;

/**
 * Creates a new person
 */
Person::Person() {
	frames = 0;
    trajectory = new Trajectory();
	personCluster = NULL;
	color = new Point();
	id = Person::nextID++;
}


/**
 * Disposes the trajectory, releasing all resources.
 */
Person::~Person() {
  delete trajectory;
	delete personCluster;
	delete color;
}

void Person::setHSVColor(float h, float s, float v) {
	color->x = h;
	color->y = s;
	color->z = v; 
}

void Person::setRGBColor(float r, float g, float b) {
	Macros::RGBtoHSV(r,g,b,color);
}
