/*


Class, methods, and variables definitions.

Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/


#ifndef mSP1_HEADEgh
#define mSP1_HEADEgh

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <cmath>
#include <stdlib.h>
#include <vector>

#include <boost/tokenizer.hpp>
#include <boost/thread/thread.hpp>

#include "gSLICr_Lib/gSLICr.h"


#include "NVTimer.h"



//ROS ROS ROS ROS ROS
#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "nodelet/nodelet.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_msgs/Vertices.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/keyboard_event.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <vector>
#include <iomanip>
#include <stddef.h>

#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//mavros

#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
//Custom messages

#include <sdk/drone_MSG.h> //Contains the Custom message
#include <sdk/drone_ver.h>
#include <sdk/e_msgs.h>


// Sample includes
#include <thread>
#include <mutex>

#include <boost/make_shared.hpp>

#include <signal.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

using namespace std;



class ThreadSafeImage
{
  boost::mutex mutex_;
  boost::condition_variable condition_;
  cv::Mat image_;

public:
  void set(const cv::Mat& image){
  boost::unique_lock<boost::mutex> lock(mutex_);
  image_ = image;
  condition_.notify_one();
};

  cv::Mat get(){
  boost::unique_lock<boost::mutex> lock(mutex_);
  return image_;
};

  cv::Mat pop(){
	cv::Mat image;
	{
		boost::unique_lock<boost::mutex> lock(mutex_);
		while (image_.empty())
		{
		condition_.wait(lock);
		}
		image = image_;
		image_.release();
	}
	return image;
  };
};







class sp1
{
public:
	sp1(int argc, char** argv);

	~sp1();
	bool sp1Loop(void);

	//CALLBACKS

	void leftCB(const sensor_msgs::ImageConstPtr& msg);
	void depthCB(const sensor_msgs::ImageConstPtr& msg);
	void odomCB(const nav_msgs::Odometry& data);
	void pclCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void keyboard_cbs(const pcl::visualization::KeyboardEvent &event, void* junk);
	bool pclYa(void);

	///ERRORS

private:
	cv::VideoWriter noseg;
	cv::VideoWriter DabOut;
	//CBS getFlagsState
	bool OdomMSG_ 	= false;
	bool LeftMSG_ 	= false;
	bool PclMSG_ 	= false;
	bool DepthMSG_ 	= false;

	//Subscribers

	image_transport::Subscriber leftS_;
	image_transport::Subscriber depthS_;
	ros::Subscriber cloudS_;
	ros::Subscriber odomS_;
	ros::Subscriber odomgpsS_;

	float weight;

	//MAVROS control
	geometry_msgs::TwistStamped U_control;

	geometry_msgs::Point Goal;

	bool frontObstacle=false;

	float ref_abs;

	bool poseflag=false;

	//Error variables
	float ref_x;
	float ref_y;
	float err_dist=5;//So it doesn't go into a false positive

	//Threshold variables
	float thd = 0.6;
	float tha = 0.6;

	//Angule variables
	float alpha; //Absolute yaw angle where north is 0º or 360º
	Eigen::Vector3f euler;//Heading Angle in degrees

	//MAVROS control

	float res = 10;
	int pi2;

	ros::Time t;
	char key;

	cv::Mat leftImage;

	int size;
	std::thread zed_callback;

	bool stop_signal;
	bool has_data;

/////////////////////////////////////////////////////////////////////////
int width;
int height;
cv::Mat izQ4;
cv::Mat izQ;
cv::Mat m_ad;
cv::Mat frame2;
cv::Mat frame;
cv::Mat boundry_draw_frame;
cv::Size s;
cv::Mat parcial;

cv::Mat depth_f;
cv::Mat depth_u;

cv::Mat izQLab;

cv::Mat Dab;

int group;
int modo;

//ROS
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber subL;
	image_transport::Subscriber subD;

	ros::Subscriber cloud_;


//DS
std::vector<int> kxy;
	std::vector<double> kr;
	std::vector<double> kg;
	std::vector<double> kb;
	std::vector<std::vector<int>> cclist;
std::vector<double> avg_r;
std::vector<double> avg_g;
std::vector<double> avg_b;

std::vector<std::vector<int>> conn_com;


std::vector<std::vector<int>>    fillmat;
	std::vector<std::vector<double>> col_r;
	std::vector<std::vector<double>> col_g;
	std::vector<std::vector<double>> col_b;


gSLICr::objects::settings my_settings;
	gSLICr::engines::core_engine* gSLICr_engine;
	gSLICr::UChar4Image* in_img;
gSLICr::UChar4Image* out_img;

const gSLICr::IntImage* Idx_im;
	const int* data_ptr;

	int im_sz;

	StopWatchInterface *my_timer,*mt2;

	int nc;
	int nc_1;
	int Dnc;

	//PCL
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_clouds;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_cloud_hulls;
	std::vector<int> largest_clouds;
	std::vector<int> largest_clouds_valid;

	char name_ch[50];

	std::vector<std::string> cloud_names; //cloud_names.resize(0);


	//MAP
	std::vector<std::vector<pcl::Vertices>> all_polygons;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_hulls;
	std::vector<pcl::Vertices> polygons;
	float r_ply,g_ply,b_ply;

	//TO STATION
	sensor_msgs::PointCloud2 hull_ros_msg;
	ros::Time t_ros;

	geometry_msgs::Point Pos_ros_msg;
	geometry_msgs::Quaternion Qua_ros_msg;

	ros::Publisher pub_hull_to_station;
	ros::Publisher pub_errors;

	Eigen::Affine3f matAff;

	Eigen::Matrix3f mat3;
	Eigen::Matrix4f posp;

	float sum_error=0;

	int p_count=0;

	//check_for_obstacles

	bool foundObstacle=false;
	//Eigen::Vector3f d_obstacle;

	//MAP VARIABLES

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

 	float resolution = 0.1;
 	int map_c_counter;
 	std::vector<std::string> map_names;
 	bool flag_pause;
    pcl::PCLPointCloud2 sHull2;
    double Xd,Yd,Zd,qx,qy,qz,qw;

    bool first_o=false;
    bool first_p=false;

    std::vector<float> errors;
    std::vector<int> num_po;

    bool yaHay=false;
    bool yaHay2=false;

	std::vector<pcl::Vertices> polygons_cam;
	cv::Mat coltest;
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_map;
	pcl::PointXYZ searchPoint;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//buscador aarbol
    float radius = 1.0f;

    bool kdfirts = true;
	std::vector<geometry_msgs::Point> centroids;
	geometry_msgs::Point pp;
	ThreadSafeImage queued_image_;
};



#endif
