//
//  Person.h
//  PeopleDetector
//
//  Created by Samantha Chen on 3/3/15.
//

#ifndef __Person__
#define __Person__

#include <pcl/point_types.h>
#include <vector>
#include "Trajectory.h"
#include "Macros.h"
#include <pcl/people/ground_based_people_detection_app.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * Model class representing a person.
 *
 */
class Person {
private:

	int frames;

	/** The ID of the person */
	int id;

    /** The trajectory of the person */
    Trajectory* trajectory;

	/** Person cluster of the person */
	pcl::people::PersonCluster<PointT>* personCluster;

	/** Average HSV color of the person */
	Point* color;
    
public:
	/** The ID to assign to the next person */
	static int nextID;
    /**
     * Creates a new Person
     */
    Person();
    
    /**
     * Disposes the Person, releasing all resources.
     */
    ~Person();
   
	 /**
     * Returns the person's ID.
     *
     * @return The ID.
     */
    int   getID()           {	return id;      }
 
    /**
     * Returns the person's trajectory 
     *
     * @return The trajectory.
     */
    Trajectory*   getTrajectory()           {	return trajectory;      }

	int getFrames() { return frames; }
	void setFrames(int v) { frames = v; }  
	Point* getColor() { return color; }
	void setHSVColor(float h, float s, float v);
	void setRGBColor(float r, float g, float b);

};

#endif /* defined(__Person__) */
