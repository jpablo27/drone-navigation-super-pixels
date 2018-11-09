
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>


/*
INS: cloud
OUTS: inliers, coefficients

This method estimates the plane coefficients given a set of points.


Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx    
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/



void planextraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointIndices::Ptr& inliers,
		pcl::ModelCoefficients::Ptr& coefficients){
	//create cloud data
	  //pcl::PointIndices::Ptr inliers_ransac_b (new pcl::PointIndices);//
	  //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);


	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZRGB> seg;//
	  // Optional
	  //seg.setOptimizeCoefficients (true);//
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PLANE);//
	  seg.setMethodType (pcl::SAC_RANSAC);//
	  seg.setMaxIterations(100);

	  seg.setDistanceThreshold (0.15);//0.06 //3.58

	  // Create the filtering object

	  seg.setInputCloud (cloud);
	  seg.segment (*inliers, *coefficients);


}
