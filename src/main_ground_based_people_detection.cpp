/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a Kinect live stream.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */
  
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include "Trajectory.h"
#include "Person.h"
#include "Macros.h"
#include <iostream>
#include <fstream>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer* viewer;

// Mutex: //
boost::mutex cloud_mutex;

enum { COLS = 640, ROWS = 480 };

std::vector<Person*>* people;
std::vector<Person*>* finishedTracking;

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm     <path_to_svm_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "*******************************************************" << std::endl;
  return 0;
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

void calcAvgColor (pcl::PointCloud<PointT>::Ptr cloud, pcl::people::PersonCluster<PointT>* pc, Point* color) {
    float h_avg = 0.0f, s_avg = 0.0f, v_avg = 0.0f;
	float total = 0.0f;		
    for (std::vector<int>::const_iterator pit = pc->getIndices().indices.begin(); pit != pc->getIndices().indices.end(); pit++){
        PointT* p = &cloud->points[*pit];
        // unpack rgb into r/g/b
        uint32_t rgb = *reinterpret_cast<int*>(&p->rgba);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;
      
        Macros::RGBtoHSV(((int)r/255.0f), ((int)g/255.0f), ((int)b/255.0f), color);

        h_avg += color->x;
		s_avg += color->y;
		v_avg += color->z;    
      
    	total++;
    }

    h_avg = (h_avg / total);
    s_avg = (s_avg / total);
    v_avg = (v_avg / total);
	
    color->x = h_avg;
    color->y = s_avg;
    color->z = v_avg;
}


void draw (PointCloudT::Ptr cloud) {
    if (viewer->wasStopped()) {
        return;
    }
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT> (cloud, rgb, "input_cloud");
    viewer->spinOnce();
}

void drawWithBoxes (PointCloudT::Ptr cloud, std::vector<pcl::people::PersonCluster<PointT> > clusters) {
    if (viewer->wasStopped()) {
        return;
    }
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT> (cloud, rgb, "input_cloud");
    unsigned int k = 0;
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it) {         
        it->drawTBoundingBox(*viewer, k);
        k++;
    }
    viewer->spinOnce();
}

int main (int argc, char** argv) {
    if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

    // Algorithm parameters:
    std::string svm_filename = DEFAULT_SVM_PATH;
    float min_confidence = DEFAULT_MIN_CONFIDENCE;
    float min_height = DEFAULT_MIN_HEIGHT;
    float max_height = DEFAULT_MAX_HEIGHT;
    float min_width = DEFAULT_MIN_WIDTH;
    float max_width = DEFAULT_MAX_WIDTH;
    float voxel_size = DEFAULT_VOXEL_SIZE;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

    // Read if some parameters are passed from command line:
    pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
    pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
    pcl::console::parse_argument (argc, argv, "--min_h", min_height);
    pcl::console::parse_argument (argc, argv, "--max_h", max_height);

    // Read Kinect live stream:
    PointCloudT::Ptr cloud (new PointCloudT);
    bool new_cloud_available_flag = false;
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
    interface->registerCallback (f);
    interface->start ();

    // Wait for the first frame:
    while(!new_cloud_available_flag) 
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    new_cloud_available_flag = false;

    cloud_mutex.lock ();    // for not overwriting the point cloud

    // Display pointcloud:
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    
    viewer = new pcl::visualization::PCLVisualizer("PCL Viewer");   
    //viewer->addPointCloud<PointT> (cloud, rgb, "input_cloud");
    viewer->setCameraPosition(0,0,-2,0,-1,0,0);

    // MODIFIED
    PointCloudT::Ptr ground  (new pcl::PointCloud<PointT>);
    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelPlane<PointT>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<PointT> (cloud));
    pcl::RandomSampleConsensus<PointT> ransac (model_p);
    ransac.setDistanceThreshold (.015);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<PointT>(*cloud, inliers, *ground);
    PointCloudT::Ptr temp (new pcl::PointCloud<PointT>);
    temp->width = 3;
    temp->height = 1;
    temp->is_dense = true;
    temp->points.resize(temp->width * temp->height);
  
    int p1 = rand() % ground->points.size();   
    int p2 = rand() % ground->points.size();
    int p3 = rand() % ground->points.size();
    temp->points[0].x = ground->points[p1].x;
    temp->points[0].y = ground->points[p1].y;
    temp->points[0].z = ground->points[p1].z;
    temp->points[1].x = ground->points[p2].x;
    temp->points[1].y = ground->points[p2].y;
    temp->points[1].z = ground->points[p2].z;
    temp->points[2].x = ground->points[p3].x;
    temp->points[2].y = ground->points[p3].y;
    temp->points[2].z = ground->points[p3].z;

    PointCloudT::Ptr clicked_points_3d(temp);

    cloud_mutex.unlock ();    


    std::cout << "Point 1: (" << temp->points[0].x << "," << temp->points[0].y << "," << temp->points[0].z << ")" << std::endl;
    std::cout << "Point 2: (" << temp->points[1].x << "," << temp->points[1].y << "," << temp->points[1].z << ")" << std::endl;
    std::cout << "Point 3: (" << temp->points[2].x << "," << temp->points[2].y << "," << temp->points[2].z << ")" << std::endl;


    // Ground plane estimation:
    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
        clicked_points_indices.push_back(i);
    
    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
    std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;


    // Create classifier for people detection:  
    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

    // People detection app initialization:
    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
    people_detector.setVoxelSize(voxel_size);                        // set the voxel size
    people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    people_detector.setClassifier(person_classifier);                // set person classifier
    //people_detector.setHeightLimits(min_height, max_height);         // set person classifier
	people_detector.setPersonClusterLimits(min_height, max_height, min_width, max_width);
    //  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

    // For timing:
    static unsigned count = 0;
    static double last = pcl::getTime (); //sw.getTime ();
    static double previousFrame = pcl::getTime ();
	people = new std::vector<Person*>();
	finishedTracking = new std::vector<Person*>();	
	Point* closestColor = new Point();	
    ofstream myfile;

    // Main loop:
    while (!viewer->wasStopped()) {
        if (new_cloud_available_flag && cloud_mutex.try_lock ()) {   // if a new cloud is available
            new_cloud_available_flag = false;
            
            double now = pcl::getTime ();//sw.getTime ();
            double deltaTime = now - previousFrame;
            // Display average framerate:
            if (++count == 30) {
                std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
                count = 0;
                last = now;
            }

            // Perform people detection on the new cloud:
            std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
            people_detector.setInputCloud(cloud);
            people_detector.setGround(ground_coeffs);                    // set floor coefficients
            people_detector.compute(clusters);                           // perform people detection
            ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

            // Removed identified person clusters that do not meet the minimum confidence requirement
            /*for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end();) {
                if(it->getPersonConfidence() > min_confidence) {
				    ++it;
                    //Eigen::Vector3f& center = it->getTCenter();
                    //if (Macros::onKinectBoundary(center(0), center(1), center(2))) {
                      //  std::cout << "ON BOUNDARY" <<  std::endl;
                    //}                
                }
			    else {
				    it = clusters.erase(it);
			    }
                
            }*/
            /*
            // Draw point clouds and/or bounding boxes
            if (DRAW && DRAW_BOXES) {
                drawWithBoxes(cloud, clusters);
            }
            else if (DRAW) {
                draw(cloud);
            }*/
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
            viewer->addPointCloud<PointT> (cloud, rgb, "input_cloud");
            /*unsigned int k = 0;
            for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it) { 			
                Eigen::Vector3f& center = it->getTCenter();                
                if(it->getPersonConfidence() > min_confidence && Macros::onKinectBoundary(center(0), center(1), center(2))) { 
                    it->drawTBoundingBox(*viewer, k);
                    k++;				    
                    std::cout << "ON BOUNDARY" <<  std::endl;
                }
            }*/
            unsigned int k = 0;
            for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it) { 			
                it->drawTBoundingBox(*viewer, k);
                k++;				    
            }
            viewer->spinOnce();
          
		    //std::cout << "Before clusters: " << clusters.size() << std::endl;
		    //std::cout << "persons: " << people->size() << std::endl;


            // Match the PersonClusters in the current iteration to the people from the previous iteration
		    for (std::vector<Person*>::iterator pIter = people->begin(); pIter != people->end();) {			
			    Person* person = (*pIter);
			    Point* pos1 = person->getTrajectory()->getPosition();
			    Point* color1 = person->getColor();

			    // Find the closest PersonCluster in the current iteration		
			    float minScore = std::numeric_limits<float>::max();
			    std::vector<pcl::people::PersonCluster<PointT> >::iterator closest;
                Point* closestColor = new Point();
                Point* color2 = new Point();
 	 
                float weights [6] = { 1.0,1.0,1.0,0.1,0.5,0.3 };
			    for (std::vector<pcl::people::PersonCluster<PointT> >::iterator cIter = clusters.begin(); cIter != clusters.end(); ++cIter) {					    
                    Eigen::Vector3f& pos2 = cIter->getTCenter();	
                    calcAvgColor(cloud, &(*cIter), color2);				    
                    float distX = fabs(pos1->x - pos2(0)); 
                    float distY = fabs(pos1->y - pos2(1));
                    float distZ = fabs(pos1->z - pos2(2));
                    float distH = fabs(color1->x - color2->x);
                    float distS = fabs(color1->y - color2->y);
                    float distV = fabs(color1->z - color2->z);
                    float distSq = distX * distX + distY * distY + distZ * distZ;
                    float score = weights[0]*distSq + weights[3]*distH + weights[4]*distS + weights[5]*distV;                     
				    
                    //std::cout << "Dist  " << distSq << "   " << distY  <<  "  " << distZ <<  std::endl;
                    //std::cout << "Color  " << distH << "   " << distS  <<  "  " << distV <<  std::endl;
                    //std::cout << "Person: " << person->getID() << "     " << score <<  std::endl;
                    //std::cout << " " <<  std::endl;

				    if (score < minScore) {
					    minScore = score;
					    closest = cIter;
					    closestColor->x = color2->x;
                        closestColor->y = color2->y;
                        closestColor->z = color2->z;
				    }
			    }
		        
                float scoreThreshold = 5.0f;
			    // If distance is small and colors are close, match the PersonCluster
			    if (minScore < scoreThreshold) {	
				    // Update the data of the person			
				    Eigen::Vector3f& center = closest->getTCenter();
                    Point* previousPos = person->getTrajectory()->getPosition();
                    float velX = (center(0) - previousPos->x) / deltaTime;
                    float velY = (center(1) - previousPos->y) / deltaTime;
                    float velZ = (center(2) - previousPos->z) / deltaTime;
				    
                    person->getTrajectory()->addPosition(center(0), center(1), center(2));
                    person->getTrajectory()->addVelocity(velX, velY, velZ);				    
                    person->setHSVColor(closestColor->x, closestColor->y, closestColor->z);
                                        
				    myfile.open("data.txt", std::ios_base::app);
				    myfile << "Person: " << person->getID() << " x: " << center(0) << " y: " << center(1) << " z: " << center(2) <<std::endl;
				    myfile << "Color: " << person->getID() << " h: " << closestColor->x << " s: " << closestColor->y << " v: " << closestColor->z <<std::endl;
                    myfile << "Velocity: " << person->getID() << " x: " << velX << " y: " << velY << " z: " << velZ <<std::endl;
                    
                    myfile << " " <<std::endl;
				    myfile.close();	

				    // Remove the matched PersonCluster from the vector of PersonClusters
				    clusters.erase(closest);
				    //std::cout << "MATCHED   clusters remaining: " << clusters.size() << std::endl;
				    ++pIter; 			
			    }
			    else if (Macros::onKinectBoundary(person->getTrajectory()->getPosition())){	
				    // Handle people leaving the Kinect's range
				    // Remove unmatched Persons from people and move them to finishedTracking
				    pIter = people->erase(pIter);
				    std::cout << "LEFT" << std::endl;				
			    }
			    else {
				    ++pIter;
			    }
			
		    }

		    // For each unmatched PersonCluster, add a new Person to people
		    for (std::vector<pcl::people::PersonCluster<PointT> >::iterator cIter = clusters.begin(); cIter != clusters.end(); ++cIter) {				
			    Eigen::Vector3f& center = cIter->getTCenter(); 			
		        Person* person = new Person();
			    person->getTrajectory()->addPosition(center(0),center(1),center(2));
			    calcAvgColor(cloud, &(*cIter), person->getColor());
			    people->push_back(person);
			    std::cout << "CREATED PERSON" << std::endl;
		    }
		    static double previousFrame = pcl::getTime ();
		    cloud_mutex.unlock ();
        }
    }

    return 0;
}


