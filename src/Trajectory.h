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
    /** Positions in the trajectory */
    std::vector<Point*>* positions;

    /** Velocities in the trajectory */
	std::vector<Point*>* velocities;
 
    /** Colors over time*/
    std::vector<Point*>* colors;

public:
    /**
     * Creates a new empty trajectory
     */
    Trajectory();
    
    /**
     * Disposes the trajectory, releasing all resources.
     */
    ~Trajectory();

	 /**
     * Returns the velocity at the specified timestep.
     *
     * @return The velocity.
     */
    Point*   getVelocity()           {		return velocities->at(velocities->size()-1);     }
    
	/**
     * Returns the position at the specified timestep.
     *
     * @return The position.
     */
    Point*   getPosition()           {		return positions->at(positions->size()-1);		}
    
    /**
     * Returns the velocity at the specified timestep.
     *
     * @return The velocity.
     */
    Point*   getVelocity(int t)           {		return velocities->at(t);     }
    
	/**
     * Returns the position at the specified timestep.
     *
     * @return The position.
     */
    Point*   getPosition(int t)           {		return positions->at(t);		}


    std::vector<Point*>* getPositions() {return positions;}

    std::vector<Point*>* getVelocities() {return velocities;}

    std::vector<Point*>* getColors() {return colors;}
    
    /**
     * Adds a position to the trajectory.
     *
     * @param value The position.
     */
    void    addPosition(float x, float y, float z);
    
    /**
     * Adds a velocity to the trajectory.
     *
     * @return The velocity.
     */
    void   addVelocity(float x, float y, float z);

    void   addColor(Point* p);
    
   
};


#endif /* defined(__Trajectory__) */
