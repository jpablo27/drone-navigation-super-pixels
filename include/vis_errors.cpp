//visualize the planes' errors
#include <iostream>
 

#include <vector>

#include <pcl/common/common_headers.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <station_map/c_msgs.h>


void visualize_errors (boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
	std::vector<float>& errors,
    std::vector<station_map::c_msgs>& centroids,
    std::vector<int>& num_points,
    int& nc_1,
    int& nc){
	char name[20];
	char e_value[100];

	int mes_size_ = errors.size();

	for(int i=0; i<nc_1; i++){
		sprintf(name,"text%d",i);
		viewer->removeText3D(name);
	}

	for(int i=0; i<nc; i++){
		sprintf(name,"text%d",i);
        sprintf(e_value,"e = %3.4f[cm]",errors[i]/num_points[i] );

		viewer->addText3D(e_value, centroids[i], 0.15, 1.0, 0.0, 0.0,name,0);
	}

}