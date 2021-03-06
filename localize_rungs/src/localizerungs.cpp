/*
 * Author: Prashanta Gyawali

 * Description: This node is for segmenting out the rungs of the ladder in a view frame and publishing them as a message

 * Refrence: LocalizeRungs.cpp from the summer project
             cluster_localize.cpp from the Jan08 pcl project
 *
 * Usage: rosrun localize_rungs localizerungs
 *
 * Last updated: 2nd May 2013, 11:00AM
 *
 */

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

// standard C++ library includes
#include "std_msgs/String.h"
#include <cmath>

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/registration/ia_ransac.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class LocalizeRungs
{
	private:
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! We will be publishing to the "centroid_rung1_pub" topic to pass the first rung
		ros::Publisher centroid_rung1_pub_;  
		//! We will be publishing to the "centroid_rung2_pub" topic to pass the sencod rung
		ros::Publisher centroid_rung2_pub_;  

		//! we will be listening to TF transforms as well
		tf::TransformListener listener_;

	public:
		//! ROS node initialization
		LocalizeRungs(ros::NodeHandle &nh)		
		{		
			nh_ = nh;
			// Create a ROS publisher for the output geometry_msg(the centroid)
   			centroid_rung1_pub_ = nh_.advertise<geometry_msgs::PointStamped>("Centroid_Rung1_Odom_Frame", 1);
			centroid_rung2_pub_ = nh_.advertise<geometry_msgs::PointStamped>("Centroid_Rung2_Odom_Frame", 1);
		}
		
		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
		{
	
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg (*input, *cloud);
			std::cout << "PointCloud before filtering has:: " << cloud->points.size() << "data points" << std::endl;

			// ..... and downsampling the point cloud
			const float voxel_grid_size = 0.1f;
			pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
			vox_grid.setInputCloud(cloud);
			vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
			vox_grid.filter(*tempCloud);
			cloud = tempCloud;

			// Create the segmentation object for the planar model and set all the parameters
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PCDWriter writer;
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations(100);
			seg.setDistanceThreshold(0.02);

			int j = 0;
			int i = 0, nr_points = (int) cloud->points.size();
			while (cloud->points.size() > 0.3 * nr_points)
			{
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud(cloud);
				seg.segment(*inliers, *coefficients);
				if (inliers->indices.size() == 0)
				{
				    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				    break;
				}

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud(cloud);
				extract.setIndices(inliers);
				extract.setNegative(false);

				// Write the planar inliers to disk
				extract.filter(*cloud_plane);
				
				// Remove the planar inliers, extract the rest
				extract.setNegative(true);
				extract.filter(*cloud_f);
				*cloud = *cloud_f;
			}
			//ROS_INFO("inside the function");
			//std::cout << "PointCloud after filtering has:: " << cloud->points.size() << "data points" << std::endl;
			cloud->width = 1;
			cloud->height = cloud->points.size();
			cloud->points.resize(cloud->width * cloud->height);

	
			pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
			std::vector<int> myinliers;

			int counter = 0;
			int flagRetain = 0;
			// create a vector for storing the rungs centroid (storing Eigen::Vector4f)
			std::vector<Eigen::Vector4f> myrungs;
			Eigen::Vector4f firstCentroid;
			Eigen::Vector4f tempCentroid;
			firstCentroid[0] = 0;
			firstCentroid[1] = 0;
			firstCentroid[2] = 0;
			firstCentroid[3] = 0;
			double xdiff = 0;
			double ydiff = 0;
			double zdiff = 0;

			while( cloud->points.size() > 5)
			{
				// create random sample consensus object and create an appropriate model
				pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
				pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
				ransac.setDistanceThreshold(0.01);
				ransac.computeModel();
				ransac.getInliers(myinliers);

				// copies all inliers of the model computed to another PointCloud named finalCloud
				pcl::copyPointCloud<pcl::PointXYZ>(*cloud, myinliers, *finalCloud);

				finalCloud->height = 1;
				finalCloud->width = finalCloud->points.size();
				finalCloud->points.resize(finalCloud->width * finalCloud->height);

				/********************************************* test to see if the slope is greater than threshold(0.9) ******************************/

				// Trying to find the coefficients of the model from the RANSAC
				Eigen::VectorXf modelCoefficients;
				ransac.getModelCoefficients(modelCoefficients);
				//std::cout << "x:" << modelCoefficients[0] << "   y:" << modelCoefficients[1] << "   z:" << modelCoefficients[2] << std::endl; //x, y and z points in line
				//std::cout << "x_dir:" << modelCoefficients[3] << "  y_dir" << modelCoefficients[4] << "  z_dir" << modelCoefficients[5] << std::endl; // line direction in x, y and z

				Eigen::Vector4f centroid;
				// modelCoefficients[0/1/2] are x, y and z coefficients in the line eqn. modelCoefficients[3/4/5] are slope in x, y and z direction
				// slope in x direction is given by modelCoefficients[3]
				// if the slope in x direction is greater than some value (-0.2) or less than some value (0.2) retain it otherwise eliminate it
				if (modelCoefficients[3] < 0.2 || modelCoefficients[3] > -0.2)
				{
				    //std::cout << "this is a good line" << std::endl;
				    pcl::compute3DCentroid(*finalCloud, centroid);
				    //std::cout << "xc:" << centroid[0] << "   yc:" << centroid[1] << "   zc:" << centroid[2] << std::endl;
				    if(counter == 0)    // if this is the first rung centroid in the list just push
				    {
				        myrungs.push_back(centroid);
				    }
				    else
				    {
				        for (i = 0; i < myrungs.size(); i++)
				         {
				            //std::cout << "checking:" << abs(centroid[1] - myrungs[i][1]) << std::endl;
				            double check = centroid[1] - myrungs[i][1];
				            if (check < 0)
				                check = -check;
				             if (check > 0.08)
				             {
				                 flagRetain = 1;
				             }
				             else
				             {
				                 flagRetain = 0;
				                 break;
				             }
				         }
				         if (flagRetain == 1)
				         {
				             myrungs.push_back(centroid);
				         }
				    }
				    counter++;
				}
				else
				{
				    std::cout << "this is a bad line" << std::endl;
				}

				//ROS_INFO("I am here!");
				
				// Delete *finalCloud from the *cloud before next iteration
				// using double loop for comparing each points in cloud(original) and finalCloud(inliers)
				pcl::io::savePCDFileASCII("remainingcloud.pcd", *cloud);
				pcl::io::savePCDFileASCII("remaininginliercloud.pcd", *finalCloud);
				for(i = 0; i < cloud->points.size(); i++)
				{
				    for (j = 0; j < finalCloud->points.size(); j++)
				    {
				        //std::cout << "xdiff: " << abs(cloud->points[i].x - finalCloud->points[j].x) << "   ydiff: " << abs(cloud->points[i].y - finalCloud->points[j].y) << "   zdiff:" << abs(cloud->points[i].z - finalCloud->points[j].z);
				        xdiff = cloud->points[i].x - finalCloud->points[j].x;
				        ydiff = cloud->points[i].y - finalCloud->points[j].y;
				        zdiff = cloud->points[i].z - finalCloud->points[j].z;
				        if (xdiff < 0)
				            xdiff = -xdiff;
				        if (ydiff < 0 )
				            ydiff = -ydiff;
				        if (zdiff < 0)
				            zdiff = -zdiff;

				        if (xdiff < 0.001  && ydiff < 0.001 && zdiff < 0.001)
				        {
				            cloud->erase(cloud->begin()+i);                    
				        }
				    }
				}

				//ROS_INFO("I am here!!");

				cloud->height = 1;
				cloud->width = cloud->points.size();
				cloud->points.resize(cloud->width * cloud->height);
				ROS_INFO("I am inside the loop hehe!!");
				std::cout << "PointCloud inside the loop has:: " << cloud->points.size() << "data points" << std::endl;
			}

			//ROS_INFO("I am outside the loop hehe!!");
			//std::cout << "PointCloud outside the loop has:: " << cloud->points.size() << "data points" << std::endl;
			// if the size of myrungs is just 1 push (0, 0, 0) as the fist element
			if (myrungs.size() == 1)
			{
				myrungs.push_back(firstCentroid);
			}
			// else sort the points in the vector myrungs according to the second element of the each vector i.e. myrungs[i][1]
			else
			{
				for (i = 0; i < myrungs.size()-1; i++)
				{
				    for (j = i+1; j < myrungs.size(); j++)
				    {
				        if (myrungs[i][1] < myrungs[j][1]) // I changed this in descending order because the rung that is uppermost in kinect frame is the lower most in odomcombined
				        {
				            tempCentroid = myrungs[i];
				            myrungs[i] = myrungs[j];
				            myrungs[j] = tempCentroid;
				        }
				    }
				}
			}

			listener_.waitForTransform("head_mount_kinect_rgb_optical_frame", "odom_combined", ros::Time(0), ros::Duration(1.0));

			geometry_msgs::PointStamped centroid_rung1;
			centroid_rung1.header.frame_id = "head_mount_kinect_rgb_optical_frame";
			centroid_rung1.header.stamp = ros::Time();
			centroid_rung1.point.x = myrungs[0][0];
			centroid_rung1.point.y = myrungs[0][1];
			centroid_rung1.point.z = myrungs[0][2];
			ROS_INFO("point1 in kinect frame is %f %f %f", centroid_rung1.point.x, centroid_rung1.point.y, centroid_rung1.point.z);
			geometry_msgs::PointStamped centroid_odom1;
			listener_.transformPoint("odom_combined", centroid_rung1, centroid_odom1);
			ROS_INFO("point1 in odomcombined frame is %f %f %f", centroid_odom1.point.x, centroid_odom1.point.y, centroid_odom1.point.z);
			printf("");			

			geometry_msgs::PointStamped centroid_rung2;
			centroid_rung2.header.frame_id = "head_mount_kinect_rgb_optical_frame";
			centroid_rung2.header.stamp = ros::Time();
			centroid_rung2.point.x = myrungs[1][0];
			centroid_rung2.point.y = myrungs[1][1];
			centroid_rung2.point.z = myrungs[1][2];
			ROS_INFO("%f %f %f", centroid_rung2.point.x, centroid_rung2.point.y, centroid_rung2.point.z);
			geometry_msgs::PointStamped centroid_odom2;
			listener_.transformPoint("odom_combined", centroid_rung2, centroid_odom2);
			ROS_INFO("point2 in odomcombined frame is %f %f %f", centroid_odom2.point.x, centroid_odom2.point.y, centroid_odom2.point.z);

			// trying to transform the points from the frame of the kinect to the odom combined frame right here instead of subscribing and publishing again in a different node
			if(centroid_odom2.point.x)
			{
				if(centroid_odom1.point.x)
				{
					centroid_rung1_pub_.publish(centroid_odom1);
					centroid_rung2_pub_.publish(centroid_odom2);
				}
			}
			std::cout << std::endl;
		}
};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "localize_rungs");
    ros::NodeHandle nh;
	LocalizeRungs localize(nh);
    
	// Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/head_mount_kinect/depth/points", 20, &LocalizeRungs::cloud_cb, &localize);

    // Spin
    ros::spin();
}

