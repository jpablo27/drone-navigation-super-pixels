//ROS ROS ROS ROS ROS
#include<iostream>
#include<vector>
#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Transform.h"
# include "geometry_msgs/Quaternion.h"
# include "geometry_msgs/Vector3.h"
# include "geometry_msgs/TransformStamped.h"
#include <sdk/drone_MSG.h>

#include <pcl/common/common_headers.h>
#include <pcl_ros/point_cloud.h>

/*
INS: All
OUTS: pub_hull_to_station

This method sends the hull-polygons to the station computer. (on-ground)

Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/



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
    std::vector<int>&                largest_clouds){

    dV1.drone_hulls.clear();
    dV1.hulls_colors.red.clear();
    dV1.hulls_colors.green.clear();
    dV1.hulls_colors.blue.clear();
    dV1.largest_hulls.clear();

    dV1.hulls_colors.red = avg_r;
    dV1.hulls_colors.green = avg_g;
    dV1.hulls_colors.blue = avg_b;
    dV1.largest_hulls=largest_clouds;

    for(int i=0; i< nc; i++){
        t_ros = ros::Time::now();
        pcl::toROSMsg( *all_hulls[i] , hull_ros_msg);
        hull_ros_msg.header.frame_id = "/zed_cu_frame";
        hull_ros_msg.header.stamp = t_ros;
        hull_ros_msg.is_bigendian = false;
        hull_ros_msg.is_dense =true;
        dV1.drone_hulls.push_back(hull_ros_msg);
        dV1.drone_pose.position = Pos_ros_msg;
        dV1.drone_pose.orientation = Qua_ros_msg;

        pub_hull_to_station.publish(dV1); //--> Out to Station

    }

}
