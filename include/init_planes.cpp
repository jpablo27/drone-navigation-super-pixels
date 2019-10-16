
#include <iostream>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PolygonMesh.h>
#include <station_map/c_msgs.h>
#include "geometry_msgs/Point.h"
#include <math.h>


/*

void init_planes (pcl::PointXYZ& searchPoint,
	std::vector<station_map::c_msgs>& centroids,
	float& resolution,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& P_map,
	char* name_ch,
    boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& all_hulls,
   	cv::Mat& coltest,
   	std::vector<double>& avg_r,
    std::vector<double>& avg_g,
    std::vector<double>& avg_b,
	std::vector<int>& largest_clouds,
	float& r_ply,
    float& g_ply,
    float& b_ply,
    std::vector<std::vector<pcl::Vertices>>& all_polygons,
    std::vector<float>& errors,
    std::vector<int>& num_points

	){
	float e_prom;
	int m_s;

	for(int i=0; i<centroids.size();i++){
		e_prom = errors[i]/num_points[i];
		if(e_prom < 0.02){
			//ADD CENTROIDS TO THE P_MAP
			searchPoint.x=ceil(centroids[i].x/resolution)*resolution;
			searchPoint.y=ceil(centroids[i].y/resolution)*resolution;
			searchPoint.z=ceil(centroids[i].z/resolution)*resolution;

			//ACTUALLY MAP THE INIT PLANES
			m_s = P_map->points.size();
			sprintf(name_ch,"cloud%d",m_s);
			viewer->addPointCloud<pcl::PointXYZRGB>(all_hulls[i],name_ch,0);
			P_map->points.push_back(searchPoint);//HERE IS ADDED


			std::cout << name_ch << std::endl;

			//UPDATE ALL
			coltest.at<cv::Vec3b>(0)[2]=avg_r[largest_clouds[i]];
			coltest.at<cv::Vec3b>(0)[1]=avg_g[largest_clouds[i]];
			coltest.at<cv::Vec3b>(0)[0]=avg_b[largest_clouds[i]];

			//cvtColor(coltest, coltest, CV_Lab2RGB);
			r_ply = float(coltest.at<cv::Vec3b>(0)[2])/255;
			g_ply = float(coltest.at<cv::Vec3b>(0)[1])/255;
			b_ply = float(coltest.at<cv::Vec3b>(0)[0])/255;
			viewer->updatePointCloud<pcl::PointXYZRGB>(all_hulls[i], name_ch);
			viewer->updatePolygonMesh<pcl::PointXYZRGB>(all_hulls[i],all_polygons[i],name_ch);
			viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r_ply,g_ply,b_ply,name_ch );
		}
	}
}*/

void init_planes (pcl::PointXYZ& searchPoint,
    std::vector<geometry_msgs::Point>& centroids,
	float& resolution,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& P_map,
	char* name_ch,
    boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& all_hulls,
   	cv::Mat& coltest,
   	std::vector<double>& avg_r,
    std::vector<double>& avg_g,
    std::vector<double>& avg_b,
	std::vector<int>& largest_clouds,
	float& r_ply,
    float& g_ply,
    float& b_ply,
    std::vector<std::vector<pcl::Vertices>>& all_polygons,
    std::vector<float>& errors,
    std::vector<int>& num_points

	){
	float e_prom;
	int m_s;

	char e_val[20];

	for(int i=0; i<centroids.size();i++){
		e_prom = errors[i]/num_points[i];
		if(e_prom < 0.02){
			//ADD CENTROIDS TO THE P_MAP
			searchPoint.x=ceil(centroids[i].x/resolution)*resolution;
			searchPoint.y=ceil(centroids[i].y/resolution)*resolution;
			searchPoint.z=ceil(centroids[i].z/resolution)*resolution;

			// THE INIT PLANES
			m_s = P_map->points.size();
			sprintf(name_ch,"cloud%d",m_s);
			//viewer->addPointCloud<pcl::PointXYZRGB>(all_hulls[i],name_ch,0);
			P_map->points.push_back(searchPoint);//HERE IS ADDED

			viewer->addPolygonMesh<pcl::PointXYZRGB>(all_hulls[i],all_polygons[i],name_ch);

			//std::cout << name_ch << std::endl;

			//UPDATE ALL
			coltest.at<cv::Vec3b>(0)[2]=avg_r[largest_clouds[i]];
			coltest.at<cv::Vec3b>(0)[1]=avg_g[largest_clouds[i]];
			coltest.at<cv::Vec3b>(0)[0]=avg_b[largest_clouds[i]];

			//cvtColor(coltest, coltest, CV_Lab2RGB);
			r_ply = float(coltest.at<cv::Vec3b>(0)[2])/255;
			g_ply = float(coltest.at<cv::Vec3b>(0)[1])/255;
			b_ply = float(coltest.at<cv::Vec3b>(0)[0])/255;
			/*viewer->updatePointCloud<pcl::PointXYZRGB>(all_hulls[i], name_ch);
			viewer->updatePolygonMesh<pcl::PointXYZRGB>(all_hulls[i],all_polygons[i],name_ch);*/
			viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r_ply,g_ply,b_ply,name_ch );

						
			//VisError
			sprintf(e_val,"e = %3.4f[cm]",e_prom);
			strcat(name_ch,"txt");
			viewer->addText3D(e_val, centroids[i], 0.15, 1.0, 0.0, 0.0,name_ch,0);

			//VisError
		}
	}
}
