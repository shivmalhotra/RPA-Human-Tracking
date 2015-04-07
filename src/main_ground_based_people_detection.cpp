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
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

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

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
}; 

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

int main (int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

  // Algorithm parameters:
  std::string svm_filename = DEFAULT_SVM_PATH;
  float min_confidence = DEFAULT_MIN_CONFIDENCE;
  float min_height = DEFAULT_MIN_HEIGHT;
  float max_height = DEFAULT_MAX_HEIGHT;
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
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);


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

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  //people_detector.setHeightLimits(min_height, max_height);         // set person classifier
	people_detector.setPersonClusterLimits(min_height, max_height, 0.3, 2.0);
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

  // For timing:
  static unsigned count = 0;
  static double last = pcl::getTime (); //sw.getTime ();

	people = new std::vector<Person*>();
	finishedTracking = new std::vector<Person*>();	
	Point* closestColor = new Point();	

  // Main loop:
  while (!viewer.wasStopped())
  {
    if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
    {
      new_cloud_available_flag = false;

      // Perform people detection on the new cloud:
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection

      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

		ofstream myfile;
      // Draw cloud and people bounding boxes in the viewer:
      viewer.removeAllPointClouds();
      viewer.removeAllShapes();
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
      viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
      unsigned int k = 0;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end();)
      {
        if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBox(viewer, k);
          k++;
			
			//Eigen::Vector3f& center = it->getTCenter(); 
			
			//myfile.open("data_clusters.txt", std::ios_base::app);
			//myfile << " x: " << center(0) << " y: " << center(1) << " z: " << center(2) <<std::endl;
			//myfile.close();
			++it;
        }
		else {
			it = clusters.erase(it);
		}
      }


		//myfile.open("data_clusters.txt", std::ios_base::app);
		//myfile << " " <<std::endl;
		//myfile.close();
      //std::cout << k << " people found" << std::endl;
      viewer.spinOnce();

      // Display average framerate:
      if (++count == 30)
      {
        double now = pcl::getTime ();//sw.getTime ();
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
      
		//std::cout << "Before clusters: " << clusters.size() << std::endl;
		//std::cout << "persons: " << people->size() << std::endl;


		// Match the PersonClusters in the current iteration to the people from the previous iteration
		for (std::vector<Person*>::iterator pIter = people->begin(); pIter != people->end();) {			
			Person* person = (*pIter);
			Point* c1 = person->getTrajectory()->getPosition();
			Point* color = person->getColor();

			// Find the closest PersonCluster in the current iteration		
			float minDist = std::numeric_limits<float>::max();
			std::vector<pcl::people::PersonCluster<PointT> >::iterator closest;
			closestColor->x = -1; closestColor->y = -1; closestColor->z = -1;		
			for (std::vector<pcl::people::PersonCluster<PointT> >::iterator cIter = clusters.begin(); cIter != clusters.end(); ++cIter) {				
				Eigen::Vector3f& c2 = cIter->getTCenter();	
				float dist = (c1->x-c2(0))*(c1->x-c2(0)) + (c1->y-c2(1))*(c1->y-c2(1)) + (c1->z-c2(2))*(c1->z-c2(2));				
				if (dist < minDist) {
					minDist = dist;
					closest = cIter;
					calcAvgColor(cloud, &(*closest), closestColor);				
				}
			}
		
			// If distance is small and colors are close, match the PersonCluster
			if (minDist < DIST_RANGE && (!COLOR_MATCHING_ON || (fabs(color->x - closestColor->x) < H_RANGE 
			&& fabs(color->y - closestColor->y) < S_RANGE && fabs(color->z - closestColor->z) < V_RANGE))) {	
				// Update the data of the person			
				Eigen::Vector3f& center = closest->getTCenter();
				person->getTrajectory()->addPosition(center(0), center(1), center(2));
				person->setHSVColor(closestColor->x, closestColor->y, closestColor->z);
				
        Macros::printInfo(person);
        //myfile.open("data.txt", std::ios_base::app);
				//myfile << "Person: " << person->getID() << " x: " << center(0) << " y: " << center(1) << " z: " << center(2) <<std::endl;
				//myfile << "Color: " << person->getID() << " h: " << closestColor->x << " s: " << closestColor->y << " v: " << closestColor->z <<std::endl;
				//myfile.close();	
				// Remove the matched PersonCluster from the vector of PersonClusters
				clusters.erase(closest);
				//std::cout << "MATCHED   clusters remaining: " << clusters.size() << std::endl;
				//person->setFrames(0);
				++pIter; 			
			}
			else if (Macros::onKinectBoundary(person->getTrajectory()->getPosition())){	
				// Handle people leaving the Kinect's range
				// Remove unmatched Persons from people and move them to finishedTracking
				//finishedTracking->push_back(person);
				pIter = people->erase(pIter);
				std::cout << "LEFT" << std::endl;				
			}
			else {
				++pIter;
			}
			
		}

		//myfile.open("data.txt", std::ios_base::app);
		//myfile << " " <<std::endl;
		//myfile.close();

		// Handle new people entering the Kinect's range
		// For each unmatched PersonCluster, add a new Person to people
		for (std::vector<pcl::people::PersonCluster<PointT> >::iterator cIter = clusters.begin(); cIter != clusters.end(); ++cIter) {				
			Eigen::Vector3f& center = cIter->getTCenter(); 			
			//if (Macros::onKinectBoundary(center(0), center(1), center(2))) {			
				Person* person = new Person();
				person->getTrajectory()->addPosition(center(0),center(1),center(2));
				calcAvgColor(cloud, &(*cIter), person->getColor());
				people->push_back(person);
				std::cout << "CREATED PERSON" << std::endl;
			//}
		}
		

		cloud_mutex.unlock ();
    }

  }
  return 0;
}


