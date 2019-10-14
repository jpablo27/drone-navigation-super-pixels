#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PolygonMesh.h>
#include <defs.h>
#include <pcl/common/centroid.h>
#include <cmath>
#include <pcl/common/transforms.h>
#include <sdk/c_msgs.h>

/*
INS: all_clouds, largest_clouds
OUTS: res_hull, polygons

This method filters the point-cloud, calculates its RANSAC plane and so the convex/concave Hull of such cloud.

Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/




using namespace std;



void get_polygons(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>        all_clouds,
    int                                                                 largest_clouds,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr&                            res_hull,
	std::vector<pcl::Vertices>&                                        polygons,
    Eigen::Affine3f& matAff,
    float& sum_error,
    geometry_msgs::Point& pp,
    int& p_count, bool& foundObstacle){

            	pcl::PassThrough<pcl::PointXYZRGB> pass;//va
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);//va
                pass.setInputCloud (all_clouds[largest_clouds]); //Filter the x field (depth)
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (0.030,10);
 /*Filter*/     pass.filter (*cloud_filtered);
                //pass.setInputCloud(cloud_filtered); //Filter the y field (width)
                //pass.setFilterFieldName ("y");
                //pass.setFilterLimits (-2.0,2.0);
                //pass.filter (*cloud_filtered);
               // pass.setInputCloud(cloud_filtered); //Filter the z field (height)
                /*
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (-1,1);
                pass.filter (*cloud_filtered);*/

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_t(new pcl::PointCloud<pcl::PointXYZRGB>);

//Deal locally with pointcloud

    pcl::transformPointCloud(*cloud_filtered,*cloud_filtered_t, matAff);
//Deal globally with point_cloud

    polygons.clear();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //Let pass only clouds with more than 20 points
                if(cloud_filtered_t->points.size()>20){
            	   planextraction(cloud_filtered_t, inliers,coefficients); //------->9th METHOD<----------------------------------------------------------------------------//
                }

    //Let only pass if there are more than 3 inliers from the plane-extraction which is the minimum to create a plane.
    if (inliers->indices.size() > 3 ){


        Eigen::Vector4f centroid;

        pcl::compute3DCentroid<pcl::PointXYZRGB>(*cloud_filtered_t,centroid);


        //PlaneGlobalPosition
        pp.x = centroid[0];
        pp.y = centroid[1];
        pp.z = centroid[2];
        //				//check_for_obstacles(pp,matAff);
        //DroneGlobalPosition
        //matAff(0,3);
        //matAff(1,3);
        //matAff(2,3);


        float H_proj,V_proj;
        float coeff_a = coefficients->values[0];
        float coeff_b = coefficients->values[1];
        float coeff_c = coefficients->values[2];
        float coeff_d = coefficients->values[3];

        V_proj = (coeff_c);

        H_proj = sqrt(pow(coeff_a,2)+pow(coeff_b,2));

        if(V_proj > H_proj){


            //Force the unitary vector of the normal on the vertical direction
            //MEANS, the plane is horizontal
            coefficients->values[0]=0;
            coefficients->values[1]=0;
            coefficients->values[2]=1;


            if((pp.z<-0.66)){
                coefficients->values[3]= -(-0.86);
            }else{
                coefficients->values[3]= -( pp.z );
            }

            }else{


            //Force the unitary vector of the normal to lie on the horizontal plane
            //Means, the plane is vertical (TESTING VERTICAL PLANES AS OBSTACLES)
            coefficients->values[0] = (coeff_a)/H_proj;
            coefficients->values[1] = (coeff_b)/H_proj;
            coefficients->values[2]=0;
            coeff_a = coefficients->values[0];
            coeff_b = coefficients->values[1];
            coefficients->values[3]= -( coeff_a*pp.x + coeff_b*pp.y );

            Eigen::Vector3f d_obstacle;

            d_obstacle << (pp.x-matAff(0,3)),(pp.y-matAff(1,3)),0;

            if (d_obstacle.norm()<1.5) {//squared distance (2m)
              /* code */
              std::cout << "Found possible obstacle" << d_obstacle.norm() << '\n';
              foundObstacle = true;
            }

        }





    	pcl::ProjectInliers<pcl::PointXYZRGB> proj;//va
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);//va


    	            proj.setModelType (pcl::SACMODEL_PLANE);
/*Projection*/    	proj.setInputCloud (cloud_filtered_t);
/*  to plane*/    	proj.setIndices (inliers);
    	            proj.setModelCoefficients (coefficients);
    	            proj.filter (*cloud_projected);


                    pcl::ConcaveHull<pcl::PointXYZRGB> chull;//va
/*Hull extraction*/ chull.setInputCloud (cloud_projected);
                    //chull.setComputeAreaVolume ( true );
                    chull.setAlpha(10);
                    chull.reconstruct (*res_hull, polygons);

        float error=0;
        sum_error=0;

        for (int i = 0; i <cloud_filtered_t->size(); i+=2)
        {
            /* code */

            error = pcl::pointToPlaneDistance(cloud_filtered_t->points[i],
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2],
                coefficients->values[3]
                );

            sum_error+=pow(error,2);
            p_count++;
        }
    }
}

void get_polygons(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>        all_clouds,
    int                                                                 largest_clouds,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr&                            res_hull,
	std::vector<pcl::Vertices>&                                        polygons,
    Eigen::Affine3f& matAff,
    float& sum_error,
    sdk::c_msgs& pp,
    int& p_count, bool& foundObstacle){

            	pcl::PassThrough<pcl::PointXYZRGB> pass;//va
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);//va
                pass.setInputCloud (all_clouds[largest_clouds]); //Filter the x field (depth)
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (0.030,10);
 /*Filter*/     pass.filter (*cloud_filtered);
                //pass.setInputCloud(cloud_filtered); //Filter the y field (width)
                //pass.setFilterFieldName ("y");
                //pass.setFilterLimits (-2.0,2.0);
                //pass.filter (*cloud_filtered);
               // pass.setInputCloud(cloud_filtered); //Filter the z field (height)
                /*
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (-1,1);
                pass.filter (*cloud_filtered);*/

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_t(new pcl::PointCloud<pcl::PointXYZRGB>);

//Deal locally with pointcloud

    pcl::transformPointCloud(*cloud_filtered,*cloud_filtered_t, matAff);
//Deal globally with point_cloud

    polygons.clear();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //Let pass only clouds with more than 20 points
                if(cloud_filtered_t->points.size()>20){
            	   planextraction(cloud_filtered_t, inliers,coefficients); //------->9th METHOD<----------------------------------------------------------------------------//
                }

    //Let only pass if there are more than 3 inliers from the plane-extraction which is the minimum to create a plane.
    if (inliers->indices.size() > 3 ){


        Eigen::Vector4f centroid;

        pcl::compute3DCentroid<pcl::PointXYZRGB>(*cloud_filtered_t,centroid);


        //PlaneGlobalPosition
        pp.x = centroid[0];
        pp.y = centroid[1];
        pp.z = centroid[2];
        //				//check_for_obstacles(pp,matAff);
        //DroneGlobalPosition
        //matAff(0,3);
        //matAff(1,3);
        //matAff(2,3);


        float H_proj,V_proj;
        float coeff_a = coefficients->values[0];
        float coeff_b = coefficients->values[1];
        float coeff_c = coefficients->values[2];
        float coeff_d = coefficients->values[3];

        V_proj = (coeff_c);

        H_proj = sqrt(pow(coeff_a,2)+pow(coeff_b,2));

        if(V_proj > H_proj){


            //Force the unitary vector of the normal on the vertical direction
            //MEANS, the plane is horizontal
            coefficients->values[0]=0;
            coefficients->values[1]=0;
            coefficients->values[2]=1;


            if((pp.z<-0.66)){
                coefficients->values[3]= -(-0.86);
            }else{
                coefficients->values[3]= -( pp.z );
            }

            }else{


            //Force the unitary vector of the normal to lie on the horizontal plane
            //Means, the plane is vertical (TESTING VERTICAL PLANES AS OBSTACLES)
            coefficients->values[0] = (coeff_a)/H_proj;
            coefficients->values[1] = (coeff_b)/H_proj;
            coefficients->values[2]=0;
            coeff_a = coefficients->values[0];
            coeff_b = coefficients->values[1];
            coefficients->values[3]= -( coeff_a*pp.x + coeff_b*pp.y );

            Eigen::Vector3f d_obstacle;

            d_obstacle << (pp.x-matAff(0,3)),(pp.y-matAff(1,3)),0;

            if (d_obstacle.norm()<1.5) {//squared distance (2m)
              /* code */
              std::cout << "Found possible obstacle" << d_obstacle.norm() << '\n';
              foundObstacle = true;
            }

        }





    	pcl::ProjectInliers<pcl::PointXYZRGB> proj;//va
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);//va


    	            proj.setModelType (pcl::SACMODEL_PLANE);
/*Projection*/    	proj.setInputCloud (cloud_filtered_t);
/*  to plane*/    	proj.setIndices (inliers);
    	            proj.setModelCoefficients (coefficients);
    	            proj.filter (*cloud_projected);


                    pcl::ConcaveHull<pcl::PointXYZRGB> chull;//va
/*Hull extraction*/ chull.setInputCloud (cloud_projected);
                    //chull.setComputeAreaVolume ( true );
                    chull.setAlpha(10);
                    chull.reconstruct (*res_hull, polygons);

        float error=0;
        sum_error=0;

        for (int i = 0; i <cloud_filtered_t->size(); i+=2)
        {
            /* code */

            error = pcl::pointToPlaneDistance(cloud_filtered_t->points[i],
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2],
                coefficients->values[3]
                );

            sum_error+=pow(error,2);
            p_count++;
        }
    }
}

void get_polygons(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>        all_clouds,
    int                                                                 largest_clouds,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr&                            res_hull,
	std::vector<pcl::Vertices>&                                        polygons,
    Eigen::Affine3f& matAff,
    float& sum_error,
    int& p_count, bool& foundObstacle){

            	pcl::PassThrough<pcl::PointXYZRGB> pass;//va
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);//va
                pass.setInputCloud (all_clouds[largest_clouds]); //Filter the x field (depth)
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (0.030,10);
 /*Filter*/     pass.filter (*cloud_filtered);
                //pass.setInputCloud(cloud_filtered); //Filter the y field (width)
                //pass.setFilterFieldName ("y");
                //pass.setFilterLimits (-2.0,2.0);
                //pass.filter (*cloud_filtered);
               // pass.setInputCloud(cloud_filtered); //Filter the z field (height)
                /*
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (-1,1);
                pass.filter (*cloud_filtered);*/

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_t(new pcl::PointCloud<pcl::PointXYZRGB>);

//Deal locally with pointcloud

    pcl::transformPointCloud(*cloud_filtered,*cloud_filtered_t, matAff);
//Deal globally with point_cloud

    polygons.clear();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //Let pass only clouds with more than 20 points
                if(cloud_filtered_t->points.size()>20){
            	   planextraction(cloud_filtered_t, inliers,coefficients); //------->9th METHOD<----------------------------------------------------------------------------//
                }

    //Let only pass if there are more than 3 inliers from the plane-extraction which is the minimum to create a plane.
    if (inliers->indices.size() > 3 ){


        Eigen::Vector4f centroid;

        pcl::compute3DCentroid<pcl::PointXYZRGB>(*cloud_filtered_t,centroid);


        //PlaneGlobalPosition
       
        //				//check_for_obstacles(pp,matAff);
        //DroneGlobalPosition
        //matAff(0,3);
        //matAff(1,3);
        //matAff(2,3);


        float H_proj,V_proj;
        float coeff_a = coefficients->values[0];
        float coeff_b = coefficients->values[1];
        float coeff_c = coefficients->values[2];
        float coeff_d = coefficients->values[3];

        V_proj = (coeff_c);

        H_proj = sqrt(pow(coeff_a,2)+pow(coeff_b,2));

        if(V_proj > H_proj){


            //Force the unitary vector of the normal on the vertical direction
            //MEANS, the plane is horizontal
            coefficients->values[0]=0;
            coefficients->values[1]=0;
            coefficients->values[2]=1;


            if((centroid[2]<-0.66)){
                coefficients->values[3]= -(-0.86);
            }else{
                coefficients->values[3]= -( centroid[2] );
            }

            }else{


            //Force the unitary vector of the normal to lie on the horizontal plane
            //Means, the plane is vertical (TESTING VERTICAL PLANES AS OBSTACLES)
            coefficients->values[0] = (coeff_a)/H_proj;
            coefficients->values[1] = (coeff_b)/H_proj;
            coefficients->values[2]=0;
            coeff_a = coefficients->values[0];
            coeff_b = coefficients->values[1];
            coefficients->values[3]= -( coeff_a*centroid[0] + coeff_b*centroid[1] );

            Eigen::Vector3f d_obstacle;

            d_obstacle << (centroid[0]-matAff(0,3)),(centroid[1]-matAff(1,3)),0;

            if (d_obstacle.norm()<1.5) {//squared distance (2m)
              /* code */
              std::cout << "Found possible obstacle" << d_obstacle.norm() << '\n';
              foundObstacle = true;
            }

        }





    	pcl::ProjectInliers<pcl::PointXYZRGB> proj;//va
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);//va


    	            proj.setModelType (pcl::SACMODEL_PLANE);
/*Projection*/    	proj.setInputCloud (cloud_filtered_t);
/*  to plane*/    	proj.setIndices (inliers);
    	            proj.setModelCoefficients (coefficients);
    	            proj.filter (*cloud_projected);


                    pcl::ConcaveHull<pcl::PointXYZRGB> chull;//va
/*Hull extraction*/ chull.setInputCloud (cloud_projected);
                    //chull.setComputeAreaVolume ( true );
                    chull.setAlpha(10);
                    chull.reconstruct (*res_hull, polygons);

        float error=0;
        sum_error=0;

        for (int i = 0; i <cloud_filtered_t->size(); i+=2)
        {
            /* code */

            error = pcl::pointToPlaneDistance(cloud_filtered_t->points[i],
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2],
                coefficients->values[3]
                );

            sum_error+=pow(error,2);
            p_count++;
        }
    }
}
