
/*


Headers needed.


Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/

#ifndef DEF_S
#define DEF_S
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
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PolygonMesh.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <geometry_msgs/PolygonStamped.h>


//ROS ROS ROS ROS ROS
#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Transform.h"
# include "geometry_msgs/Quaternion.h"
# include "geometry_msgs/Vector3.h"
# include "geometry_msgs/TransformStamped.h"
#include <sdk/drone_MSG.h>
#include <sdk/c_msgs.h>


using namespace std;
//1st Method
void create_connection_ref_mat(vector<vector<int>>& con_mat, int k);

//2nd Method
void find_sp_coordinates(const int* &klabels,int& im_sz, vector<vector<int>>&fillmat, cv::Mat img,
		vector<vector<double>>& col_r,
				vector<vector<double>>& col_g,
				vector<vector<double>>& col_b);

//3rd Method
void create_adj_mat_mask(const int*klabels,cv::Mat& ad_mat,int height,int width);

//4th Method
void extract_colours(
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<vector<double>> col_r,
		vector<vector<double>> col_g,
		vector<vector<double>> col_b
		);
//5th Method
void connected_components_listDAB(
		cv::Mat						    ad_mat,
		vector<vector<int>>& 		conexiones,
		vector<vector<int>>& 		cclist,
		int 						sz,
		int& 						group_i,
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<double>&				avg_r,
		vector<double>&				avg_g,
		vector<double>&				avg_b
		);
void connected_components_listLAB(
		cv::Mat						    ad_mat,
		vector<vector<int>>& 		conexiones,
		vector<vector<int>>& 		cclist,
		int 						sz,
		int& 						group_i,
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<double>&				avg_r,
		vector<double>&				avg_g,
		vector<double>&				avg_b
		);
void connected_components_listDDD(
		cv::Mat						    ad_mat,
		vector<vector<int>>& 		conexiones,
		vector<vector<int>>& 		cclist,
		int 						sz,
		int& 						group_i,
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<double>&				avg_r,
		vector<double>&				avg_g,
		vector<double>&				avg_b
		);
//6th Method

void fill_tool(
		cv::Mat&					img_seg2,
		vector<vector<int>>& 	cclist,
		vector<double>& 		avg_r,
		vector<double>& 		avg_g,
		vector<double>& 		avg_b,
		vector<vector<int>>&	fillmat
		);
//7th Method

void fill_pcl_AGL(
		vector<vector<int>>& 								cclist,
		vector<vector<int>>&								fillmat,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr 				mcloud,
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>&      allclouds,
		vector<int>& largestAGLs
		);
//8th Method
void get_polygons(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_clouds,
		int largest_clouds,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& res_hull,
		std::vector<pcl::Vertices>& polygons,
		Eigen::Affine3f& matAff,
		float& sum_error,
		sdk::c_msgs& pp,
		int& p_count,
		bool& foundObstacle);
//9th Method
void planextraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointIndices::Ptr& inliers,
		pcl::ModelCoefficients::Ptr& coefficients);
//10th Method
void add_remove_polygons(int& 								nc_1,
	int& 													Dnc,
	char* 													name_ch,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& 	all_hulls,
	std::vector<std::string>& 								cloud_names);
//11th Method
void send_hulls(sdk::drone_MSG& dV1,
	int& nc,
	ros::Time& t_ros,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& all_hulls,
	sensor_msgs::PointCloud2& hull_ros_msg,
	ros::Publisher& pub_hull_to_station,
	geometry_msgs::Point& Pos_ros_msg,
    geometry_msgs::Quaternion& Qua_ros_msg,
    std::vector<double>&             avg_r,
    std::vector<double>&             avg_g,
    std::vector<double>&             avg_b,
    std::vector<int>&                largest_clouds);

#endif
