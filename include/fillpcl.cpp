#include <vector>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iomanip>


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>

using namespace std;
using namespace cv;

/*
INS: tails, fillmat, mcloud
OUTS: allclouds, largestAGLs

This method retrieves a dense-depth map segmentated into as many clouds as SP aglomerations.
Each cloud corresponds to a SP algomeration.

Jose-Pablo Sanchez-Rodriguez	ITESM a01161469@itesm.mx	
Jose Martinez-Carranza			INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez			ITESM aaceves@itesm.mx
*/

void fill_pcl_AGL(
		vector<vector<int>>& 								tails,
		vector<vector<int>>&								fillmat,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr 				mcloud,
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>&      allclouds,
		vector<int>& largestAGLs
		)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	int sp_size,pix, super_pixel;
	int AGLsz;
	int noAGL= tails.size();

	for(int AGL=0; AGL < noAGL; AGL++){

		AGLsz = tails[AGL].size(); //Get the size of the current agglomeration

		if(AGLsz>3){ //If the agllomeration is greater than # then we consider it, otherwise we ignore it. 
		//if(true){
			largestAGLs.push_back(AGL);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_acc (new pcl::PointCloud<pcl::PointXYZRGB>);

		for(int spi=0; spi<AGLsz;spi++){

			super_pixel=tails[AGL][spi]; //Check each SP of the current agglomeration
			sp_size=fillmat[super_pixel].size();

			cloud_temp->width    = 1;
			cloud_temp->height   = 1;
			cloud_temp->is_dense = true; //Because we're ensuring of no invalid points.
			cloud_temp->points.resize (1);

			for(int i=0; i<sp_size; i+=8){ // PCL sub-sampling
				pix=fillmat[super_pixel][i];
				cloud_temp->push_back(mcloud->points[pix]);
			}

			*cloud_acc+=*cloud_temp; //Point Cloud Concatenation or Aggregation
		}
		//Once all points are added to the agglomeration cloud, we save it in a vector.
		allclouds.push_back(cloud_acc);


	}
}
