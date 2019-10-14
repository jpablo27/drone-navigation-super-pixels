#include <def_func.h>

/*
INS: all variables
OUTS: viewer update

This method updates the viewer.


Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx    
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/


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
    ){
 
	for(int i=0; i<nc; i++){
    	//UPDATE ALL 
        coltest.at<cv::Vec3b>(0)[2]=avg_r[largest_clouds[i]];
        coltest.at<cv::Vec3b>(0)[1]=avg_g[largest_clouds[i]];
        coltest.at<cv::Vec3b>(0)[0]=avg_b[largest_clouds[i]];

        //cvtColor(coltest, coltest, CV_Lab2RGB);
        r_ply = float(coltest.at<cv::Vec3b>(0)[2])/255;
        g_ply = float(coltest.at<cv::Vec3b>(0)[1])/255;
        b_ply = float(coltest.at<cv::Vec3b>(0)[0])/255;
        viewer->updatePointCloud<pcl::PointXYZRGB>(all_hulls[i], cloud_names[i]);
        //viewer->updatePointCloudPose(cloud_names[i],matAff);//vESTAAA
        viewer->updatePolygonMesh<pcl::PointXYZRGB>(all_hulls[i],all_polygons[i],cloud_names[i]);
        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r_ply,g_ply,b_ply,cloud_names[i] );
        //Trigger for Mapping

        if(flag_map){
            sprintf(name_ch,"map%d",(i+map_c_counter));
            std::cout<< i+map_c_counter << std::endl;
            map_names.push_back(name_ch);
            
            viewer->addPointCloud<pcl::PointXYZRGB>(all_hulls[i], map_names.back(), 0);
            //viewer->updatePointCloudPose(map_names.back(),matAff); //estaaaa
            viewer->updatePolygonMesh<pcl::PointXYZRGB>(all_hulls[i],all_polygons[i],map_names.back());
            viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r_ply,g_ply,b_ply,map_names.back());
            pcl::PointXYZ center;
            center.x = matAff(0,3);
            center.y = matAff(1,3);
            center.z = matAff(2,3);
            viewer->addSphere(center,0.1,1,0,0,map_names.back());
        }

    }
}