#ifndef STAT_FUNCDD_
#define STAT_FUNCDD_


#include <iostream>

#include <pcl/point_types.h>


#include <opencv2/opencv.hpp>


#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common_headers.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PolygonMesh.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <geometry_msgs/PolygonStamped.h>
#include <station_map/c_msgs.h>


void draw_camera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ob_camera);

void add_remove_polygons(int& 								nc_1, 
	int& 													Dnc,
	char* 													name_ch, 
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& 	all_hulls,
	boost::shared_ptr<pcl::visualization::PCLVisualizer>& 	viewer, 
	std::vector<std::string>& 								cloud_names);

void add_remove_polygons(int& 										nc_1, 
	int& 															Dnc,
	char* 															name_ch, 
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& 			all_hulls,
	std::vector<std::string>& 										cloud_names);

void update_polygons_viewer(int& nc,
	cv::Mat& coltest, 
    std::vector<double>& avg_r, 
    std::vector<double>& avg_g, 
    std::vector<double>& avg_b, 
    std::vector<int>& largest_clouds, 
    float& r_ply, 
    float& g_ply, 
    float& b_ply,
    char* name_ch, 
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& all_hulls, 
    boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    int& map_c_counter,
    std::vector<std::string>& cloud_names,
    Eigen::Affine3f& matAff,
    std::vector<std::vector<pcl::Vertices>>& all_polygons,
    std::vector<std::string> map_names,
    double& Xd,
    double& Yd,
    double& Zd,
    bool& flag_map
    );

void visualize_errors (boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    std::vector<float>& errors,
    std::vector<station_map::c_msgs>& centroids,
    std::vector<int>& num_points,
    int& nc_1,
    int& nc);

#endif