//
//  Trajectory.h
//  PeopleDetector
//
//  Created by Samantha Chen on 3/3/15.
//

#ifndef __Trajectory__
#define __Trajectory__

#include <vector>
#include "Macros.h"


/**
 * Model class representing an trajectory.
 *
 */
class Trajectory {
private:

    int id;
    std::vector<Point*>* positions;
	  std::vector<Point*>* velocities;
    std::vector<Point*>* colors;
    std::vector<float>* times;

    /** Average HSV color of the person */
    Point* color;

public:
    /** The ID to assign to the next person */
    static int nextID;

    Trajectory();
    ~Trajectory();

    int getID() { return id; }

    Point* getPosition() { return positions->at(positions->size()-1); }
    Point* getVelocity() { return velocities->at(velocities->size()-1); }
    Point* getColor() { return colors->at(colors->size()-1); }
    float getTime() { return times->at(times->size()-1); }

    Point* getVelocity(int t) {	return velocities->at(t); }
    Point* getPosition(int t) {	return positions->at(t); }
    Point* getColor(int t) { return colors->at(t); }
    float getTime(int t) { return times->at(t); }

    std::vector<Point*>* getPositions() { return positions; }
    std::vector<Point*>* getVelocities() { return velocities; }
    std::vector<Point*>* getColors() { return colors; }
    std::vector<float>* getTimes() { return times; }

    void addPosition(float x, float y, float z);
    void addVelocity(float x, float y, float z);
    void addColor(float h, float s, float b);
    void addTime(float t) { times->push_back(t); }

    void addPosition(Point* p) { positions->push_back(p); }
    void addVelocity(Point* p) { velocities->push_back(p); }
    void addColor(Point* p)  { colors->push_back(p); }
  
};


#endif /* defined(__Trajectory__) */
