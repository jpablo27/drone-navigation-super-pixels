#ifndef STAT_DDD_
#define STAT_DDD_


//ROS ROS ROS ROS ROS
#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
//#include "geometry_msgs/Point.h"
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "nodelet/nodelet.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <dynamic_reconfigure/server.h>


//PCL PCL PCL

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/keyboard_event.h>
#include <pcl/kdtree/kdtree_flann.h>



//OCV OCV OCV

#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <station_map/drone_MSG.h> //Contains the Custom message
#include <station_map/drone_ver.h>
#include <station_map/e_msgs.h>
#include <station_map/c_msgs.h>

//GEN 

#include <math.h>
//FUnctions

#include <init_planes.cpp>
#include <new_planes_plot.cpp>
#endif