
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
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/Point.h>

#include <math.h>
          
void new_q_planes(pcl::PointXYZ& searchPoint,
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
    int& map_c_counter,
    pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    float& radius,    
    std::vector<float>& errors,
    std::vector<int>& num_points
){
    int kdN=0;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    int m_s;
    float e_prom;

    for(int i=0; i<centroids.size(); i++){
        e_prom = errors[i]/num_points[i];
        if(e_prom < 0.02){
            searchPoint.x=ceil(centroids[i].x/resolution)*resolution;
            searchPoint.y=ceil(centroids[i].y/resolution)*resolution;
            searchPoint.z=ceil(centroids[i].z/resolution)*resolution;

            // checar si este punto ya existe en el mapa

            kdN = kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance); // Buscar el plano
 
            
            //  Se encontró un punto && El punto corresponde al centroide del plano

            if( (kdN > 0)){//&&(pointNKNSquaredDistance[0] < 0.27) ){//Uncertainty of 0.
                // YA EXISTE (no remapear, checar con distancia que realmente sea ese punto)

            }else{
                sprintf(name_ch,"cloud%d",map_c_counter);
                viewer->addPointCloud<pcl::PointXYZRGB>(all_hulls[i],name_ch,0);
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
                // NO EXISTE AÚN(agregar al mapa)
                P_map->points.push_back(searchPoint);
                map_c_counter=P_map->points.size();
                std::cout << " NUEVO " << std::endl;
            }
        }else{
            std::cout << "ERROR: " << e_prom << std::endl;
        }
    }
}