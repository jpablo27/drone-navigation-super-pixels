#include <sp1_header.h>
#include <defs.h>
#include <cstdlib>


/*
This is actually the main file where all the methods are called and the callbacks defined.
Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/


using namespace cv;
using namespace std;

#define PI 3.14159265
// mixChannels depth and Lab---------
int from_to_d[] ={ 0,2 };
int from_to_ab[] ={ 1,0, 2,1 };
//-----------------------------------

//Only Depth-------------------------
int from_to_3f[] ={ 0,0,0,1, 0,2 };
//-----------------------------------

void mySigintHandler(int sig)
{

  ros::shutdown();
}


void load_image(const cv::Mat& inimg, gSLICr::UChar4Image* outimg)
{
	gSLICr::Vector4u* outimg_ptr = outimg->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < outimg->noDims.y;y++)
		for (int x = 0; x < outimg->noDims.x; x++)
		{
			int idx = x + y * outimg->noDims.x;
			outimg_ptr[idx].b = inimg.at<Vec3b>(y, x)[0];
			outimg_ptr[idx].g = inimg.at<Vec3b>(y, x)[1];
			outimg_ptr[idx].r = inimg.at<Vec3b>(y, x)[2];
		}
}

void load_image(const gSLICr::UChar4Image* inimg, cv::Mat& outimg)
{
	const gSLICr::Vector4u* inimg_ptr = inimg->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < inimg->noDims.y; y++)
		for (int x = 0; x < inimg->noDims.x; x++)
		{
			int idx = x + y * inimg->noDims.x;
			outimg.at<Vec3b>(y, x)[0] = inimg_ptr[idx].b;
			outimg.at<Vec3b>(y, x)[1] = inimg_ptr[idx].g;
			outimg.at<Vec3b>(y, x)[2] = inimg_ptr[idx].r;
		}
}

sp1::sp1(int argc, char** argv): it_(nh_){


	ROS_INFO("SP drone wSDK");


  //Goal.x = 0;
  //Goal.y = 20;
  Goal.z = 0;

  res = 10;
  pi2 = PI*2*res;

	width = 672;
	height = 376;

	// Allocate PCL point cloud at the resolution
	nube = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	nube->width = width;
	nube->height = height;

	size = width*height;

  // Get parameters from launch file
  nh_.getParam("/sdk/modo",modo);
  nh_.getParam("/sdk/weight",weight);
  nh_.getParam("/sdk/GoalX",Goal.x);
  nh_.getParam("/sdk/GoalY",Goal.y);

  std::cout << "GOAL: X: " << Goal.x << " Y: " << Goal.y << std::endl;

	signal(SIGINT, mySigintHandler);

  //Subscriptions
  leftS_ = it_.subscribe("/zed/left/image_rect_color",1,&sp1::leftCB,this);
  depthS_= it_.subscribe("/zed/depth/depth_registered",1,&sp1::depthCB,this);
  cloudS_= nh_.subscribe("/zed/point_cloud/cloud_registered",1, &sp1::pclCB,this);
  odomS_ = nh_.subscribe("/zed/odom",1,&sp1::odomCB,this);
  odomgpsS_=nh_.subscribe("/mavros/local_position/pose",1,&sp1::poseGPScb,this);

  //Advertisements

    //To Station
  pub_hull_to_station = nh_.advertise<sdk::drone_MSG> ("/UAV/hull", 1);
  pub_errors = nh_.advertise<sdk::e_msgs>("/UAV/errors",1);
    //To Pixhawk
  pub_control = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
  publicanube = nh_.advertise<sensor_msgs::PointCloud2>("prueba/lolol",1);

  U_control.twist.linear.x=0;
  U_control.twist.linear.y=0;
  U_control.twist.linear.z=0;
  U_control.twist.angular.x=0;
  U_control.twist.angular.y=0;
  U_control.twist.angular.z=0;

  izQ4=cv::Mat(height, width, CV_32FC1);
  izQ=cv::Mat(height, width, CV_8UC3);
  izQLab=cv::Mat(height, width, CV_8SC3 );
  Dab = cv::Mat(height, width, CV_8UC3);

  std::cout << "izqtype: " << izQ.type() << '\n';

  parcial = cv::Mat(height,width,CV_8UC3);
  //cv::namedWindow("Frame",cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("semi",cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("VIEW", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("VIEW2", cv::WINDOW_AUTOSIZE);

  group = 0;

  my_settings.img_size.x = width;
  my_settings.img_size.y = height;
  my_settings.no_segs = 200;//atoll(argv[1]);
  my_settings.spixel_size = 16;
  my_settings.coh_weight =weight; //atoll(argv[2]);
  my_settings.no_iters = 4;
  my_settings.color_space = gSLICr::CIELAB; //gSLICr::XYZ; // gSLICr::CIELAB for Lab, or gSLICr::RGB for RGB
  my_settings.seg_method = gSLICr::GIVEN_NUM;//gSLICr::GIVEN_SIZE; // or gSLICr::GIVEN_NUM for given number
  my_settings.do_enforce_connectivity = true;

  gSLICr_engine = new gSLICr::engines::core_engine(my_settings);
  im_sz = width*height;
  in_img = new gSLICr::UChar4Image(my_settings.img_size, true, true);
  out_img = new gSLICr::UChar4Image(my_settings.img_size, true, true);
  s = cv::Size(my_settings.img_size.x, my_settings.img_size.y);
  frame       =cv::Mat(s,CV_8UC3);
  boundry_draw_frame.create(s, CV_8UC3);
  frame2      =cv::Mat(s,CV_8UC3);

  //sdkCreateTimer(&my_timer);
  //sdkCreateTimer(&mt2);

  largest_clouds.resize(0);

  nc_1=0;

  matAff.setIdentity();
  posp.setIdentity();
  mat3.setIdentity();

  
  int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
  double fps = 15.0;                          // framerate of the created video stream
  string filename = "/home/pablo/vid/live.avi"; 
  string filenam2 = "/home/pablo/vid/noseg.avi";          // name of the output video fil
  cv::Size soso(672, 376);
  DabOut.open(filename,codec,fps,soso,true);
  noseg.open(filenam2,codec,fps,soso,true);

  if(!DabOut.isOpened()){
    std::cout << "No se puede grabar!" << std::endl;
  }else{
    std::cout << "Grabando!" << std::endl;
  }



}

sp1::~sp1(){

  DabOut.release();
  noseg.release();
	std::cout << "TERMINATED" << '\n';

	destroyAllWindows();
}

void sp1::leftCB(const sensor_msgs::ImageConstPtr& msg){
  LeftMSG_ = true;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  izQ = cv_ptr->image;
}

void sp1::depthCB(const sensor_msgs::ImageConstPtr& msg){
  DepthMSG_ = true;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg);
  cv::cvtColor(cv_ptr->image,depth_f, CV_RGBA2RGB);
}

void sp1::pclCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg){
  PclMSG_ = true;
	nube = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*nube);

  ///////////////
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*nube,output);
  output.header.frame_id = "zed_left_camera"; // Set the header values of the ROS message
  output.header.stamp = ros::Time::now();
  output.height = height;
  output.width = width;
  output.is_bigendian = false;
  output.is_dense = true;
  publicanube.publish(output);

}

void sp1::odomCB(const nav_msgs::Odometry& data)
{
    OdomMSG_ = true;
    Pos_ros_msg = data.pose.pose.position;
    Qua_ros_msg = data.pose.pose.orientation;


    mat3 = Eigen::Quaternionf(Qua_ros_msg.w,
        Qua_ros_msg.x,
        Qua_ros_msg.y,
        Qua_ros_msg.z).toRotationMatrix();

    posp.block(0,0,3,3) = mat3;
    matAff.matrix() = posp;


    matAff(0,3) = data.pose.pose.position.x;
    matAff(1,3) = data.pose.pose.position.y;
    matAff(2,3) = data.pose.pose.position.z;
}



//MAIN LOOP
bool sp1::sp1Loop(void){


  cvtColor(izQ,izQLab, CV_BGR2Lab); // Transform the 3 channel image into Lab color-space

//CHANGE MODE IN HEADER FILE sp1_header.h
	if (modo == 1) {
		/* ---------------------------DEPTH ONLY------------------ */
		frame = depth_f;
		load_image(frame, in_img); // SP load image method
	}else if (modo == 2) {
		/* ---------------------------MIXED CHANNELS--------------- */
		cv::mixChannels(&depth_f,1,&Dab,1,from_to_d,1);
		cv::mixChannels(&izQLab,1,&Dab,1,from_to_ab,2);
		frame = Dab;
		load_image(Dab, in_img);// SP load image method
	}else{
		/* ---------------------------LAB ONLY--------------------- */
		frame = izQLab;
		load_image(frame, in_img); // SP load image method
	}


  noseg.write(frame);


  //cv::imshow("Frame",frame);

	gSLICr_engine->Process_Frame(in_img); // Perform the SP segmentation

	gSLICr_engine->Draw_Segmentation_Result(out_img); //If you want to see the SP segmentation imshow(out_img) further below.

	load_image(out_img, boundry_draw_frame); // Create the SP segmentation on a cv::Mat type ()
  //cv::imshow("VIEW",boundry_draw_frame);

	Idx_im = gSLICr_engine->Get_Seg_Res(); // Get the SP indices

	data_ptr = Idx_im->GetData(MEMORYDEVICE_CPU); // Store data on host

	int n_sp = data_ptr[im_sz-1]+1; // Get the total number of SP

	cv::waitKey(10);

	//------>1st METHOD<----------------------------------------------------------------------------//

	create_connection_ref_mat(conn_com, n_sp);


	fillmat.resize  (n_sp);
	col_r.resize    (n_sp);
	col_g.resize    (n_sp);
	col_b.resize    (n_sp);

	//------>2nd METHOD<----------------------------------------------------------------------------//

  if(modo == 1){
    find_sp_coordinates(data_ptr, im_sz, fillmat,frame,col_r,col_g,col_b);
  }else if (modo == 2) {
    find_sp_coordinates(data_ptr, im_sz, fillmat,frame,col_r,col_g,col_b);
  }else{
    find_sp_coordinates(data_ptr, im_sz, fillmat,frame,col_r,col_g,col_b);
  }




	//------>3rd METHOD<----------------------------------------------------------------------------//

    cv::Mat m_ad = cv::Mat(n_sp,n_sp, CV_64FC1,cvScalar(0));

    create_adj_mat_mask(data_ptr,m_ad,height,width);

	//------>4th METHOD<----------------------------------------------------------------------------//

    extract_colours(kr, kg, kb, col_r, col_g, col_b);

    /* //Uncomment these lines to generate the homogeneous super-pixel image

    for(int i=0; i<im_sz;i++){
      parcial.at<cv::Vec3b>(i)[0] = double(kb[data_ptr[i]]);
      parcial.at<cv::Vec3b>(i)[1] = double(kg[data_ptr[i]]);
      parcial.at<cv::Vec3b>(i)[2] = double(kr[data_ptr[i]]);

    }

    cv::imshow("semi",parcial);*/

	//------>5th METHOD<----------------------------------------------------------------------------//

    if(modo == 1){
      connected_components_listDDD(m_ad,conn_com,cclist, n_sp,group, kr, kg, kb,avg_r,avg_g,avg_b);
    }else if(modo == 2){
      connected_components_listDAB(m_ad,conn_com,cclist, n_sp,group, kr, kg, kb,avg_r,avg_g,avg_b);
    }else{
      connected_components_listLAB(m_ad,conn_com,cclist, n_sp,group, kr, kg, kb,avg_r,avg_g,avg_b);
    }


   	//------>6th METHOD<----------------------------------------------------------------------------//

    fill_tool(frame2, cclist, avg_r, avg_g, avg_b,fillmat);

    //cv::imshow("VIEW",frame2);
    DabOut.write(frame2);


    //------>7th METHOD<----------------------------------------------------------------------------//
    		//		|		|		  |       |             |
    fill_pcl_AGL(cclist, fillmat, nube, all_clouds,largest_clouds);

    nc=largest_clouds.size();


    //Reset polygons' vectors for each iteration
    all_polygons.clear();
    all_hulls.clear();

    dV1.v_hulls.clear();
    error_vec.errors.clear();
    error_vec.centroids.clear();
    error_vec.number_op.clear();

     //From 0 to the nth cloud, and clear those not usefull
    for(int ic=0; ic<nc; ic++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_hull (new pcl::PointCloud<pcl::PointXYZRGB>); // Polygon Points

    //------->8th METHOD<----------------------------------------------------------------------------//
    //>->->->->9th METHOD inside<--<------//
    //                   |                |             |          |

        polygons.clear();
        drone_ver1.v_hulls.clear();

        sum_error = 0;
        p_count = 0;
        get_polygons(all_clouds, largest_clouds[ic], res_hull, polygons,matAff,sum_error,pp,p_count,foundObstacle); //Get the polygon points lying on a plane.


        int hull_size1=res_hull->points.size();
        if(hull_size1>3){ //If the polygon data is big enough, add it, otherwise remove it.

          largest_clouds_valid.push_back(largest_clouds[ic]);
          all_polygons.push_back(polygons);
          all_hulls.push_back(res_hull);

          pcl_conversions::fromPCL(polygons,drone_ver1.v_hulls);
          dV1.v_hulls.push_back(drone_ver1);

          error_vec.errors.push_back(sum_error);
          error_vec.centroids.push_back(pp);
          error_vec.number_op.push_back(p_count);
        }
    }

        //Dnc Difference of the number of clouds between current and previous time.
  nc=all_hulls.size();
	Dnc= nc-nc_1;

    //------->10th METHOD<----------------------------------------------------------------------------//

  add_remove_polygons(nc_1, Dnc,name_ch, all_hulls, cloud_names);

  //------->11th METHOD<---------------------------------------------------------------------------// ROS Publisher inside 11th
  pub_errors.publish(error_vec);
  send_hulls(dV1,nc, t_ros, all_hulls, hull_ros_msg, pub_hull_to_station, Pos_ros_msg, Qua_ros_msg,avg_r,avg_g,avg_b,largest_clouds_valid);

  nc_1=nc;

//CLEAR ALL
  largest_clouds_valid.clear();
	kxy.clear();//
  kr.clear();//
  kg.clear();//
  kb.clear();//
  cclist.clear();//
	avg_r.clear();//
	avg_g.clear();//
	avg_b.clear();//
	conn_com.clear();//
	fillmat.clear();//
	col_r.clear();//
	col_g.clear();//
	col_b.clear();//
	all_clouds.clear();//
  largest_clouds.clear();//

  if(foundObstacle){
    foundObstacle=false;
    return true;
  }else
    return false;

}


///////////////MAVROS FUNCTIONS/////////////////////////

bool sp1::getYawControlState(void){
  return controlOrientation;
}
/*
void sp1::YawControl(float ref_ext){
  controlOrientation = false;
  err_ang = (ref_ext - alpha);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;

  d_err = err_ang-err_prev;
  err_prev = err_ang;
  i_err+=err_ang;

  ua = kp_ang*err_ang  +  kd_ang*d_err  +  ki_ang*i_err;

  if(ua>=tha){
    ua=tha;
  }else if(ua<=-tha){
    ua=-tha;
  }


  U_control.twist.angular.z=ua;
}*/

void sp1::YawControl(float x, float y){
  controlOrientation = false;
  err_ang = (atan2((y-pose_py),(x-pose_px)) - euler[2]);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;

  d_err = err_ang-err_prev;
  err_prev = err_ang;
  i_err+=err_ang;

  ua = kp_ang*err_ang  +  kd_ang*d_err  +  ki_ang*i_err;

  if(ua>=tha){
    ua=tha;
  }else if(ua<=-tha){
    ua=-tha;
  }


  U_control.twist.angular.z=ua;
}

bool sp1::getPositionControlState(void){
  return controlPosition;
}

bool sp1::getWpControlState(void){
  return controlPosition&&controlOrientation;
}

void sp1::setOnlyRotateF(void){
  //true in control Orientation  means it won't rotate
  controlOrientation = true;
}

void sp1::setOnlyDisplaceF(void){
  //true in control Orientation  means it won't move
  controlPosition = true;
}

void sp1::setOnlyRotateT(void){
  //false in control Orientation  means it will rotate
  controlOrientation = false;
}

void sp1::setOnlyDisplaceT(void){
  //false in control Orientation  means it will move
  controlPosition = false;
}

void sp1::PositionControl(float x, float y){
  controlPosition = false;
  erry = y - pose_py;
  errx = x - pose_px;

  err_dist = sqrt(pow(erry,2)+pow(errx,2));


  d_errd = err_dist - err_prevd;
  err_prevd = err_dist;

  ud = kp_dist*err_dist + kd_dist*d_errd;

  if(ud<=0.0){
    ud = 0.0;
  }else if(ud>=thd){
    ud = thd;
  }


  U_control.twist.linear.x = ud*cos(alpha);
  U_control.twist.linear.y = ud*sin(alpha);
}

void sp1::poseGPScb(const geometry_msgs::PoseStamped& msg){

  pose_qw = msg.pose.orientation.w;
  pose_qz = msg.pose.orientation.z;
  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;
	pose_pz = msg.pose.position.z;

  euler= Eigen::Quaternionf(pose_qw,0, 0, pose_qz).toRotationMatrix().eulerAngles(0, 1, 2);
  alpha = euler[2];
  alpha = atan2(sin(alpha),cos(alpha));
  poseflag = true;
}

bool sp1::getPoseFlag(void){
  return poseflag;
}

geometry_msgs::Point sp1::getPosition(void){
  geometry_msgs::Point punto;
  punto.x = pose_px;
  punto.y = pose_py;
  punto.z = pose_pz;
  return punto;
}

geometry_msgs::Point sp1::getNewSetPoint(float angref){
  geometry_msgs::Point pt1;
  geometry_msgs::Point pt2;
  pt1 = sp1::getPosition();
  pt2.x = pt1.x + 2*cos(angref);
  pt2.y = pt1.y + 2*sin(angref);
  return pt2;
}

float sp1::getProposedWp(void){
  float wp = rand() % (pi2) ;
  wp = (wp - pi2/2)/res;
  ref_abs = wp + alpha;
  return ref_abs;
}

void sp1::setFrontObstacle(bool set){
  frontObstacle = set;
}
bool sp1::getFrontObstacle(void){
  return frontObstacle;
}

float sp1::getGlobalYaw(void){
  return alpha;
}

float sp1::getGoalYaw(void){
  return atan2((Goal.y-pose_py),(Goal.x-pose_px));
}

void sp1::publishMAVROS(void){
  U_control.header.stamp = ros::Time::now();
  pub_control.publish(U_control);
}

bool sp1::getFlagsState(void){
  if (OdomMSG_&&LeftMSG_&& PclMSG_ && DepthMSG_) {
    OdomMSG_ = false;
    LeftMSG_ = false;
    PclMSG_ = false;
    DepthMSG_ = false;
    return true;
  }else{
    return false;
  }
}

float sp1::getDistanceError(float x, float y){
  erry = y - pose_py;
  errx = x - pose_px;
  err_dist = sqrt(pow(erry,2)+pow(errx,2));
  return err_dist;
}

float sp1::getHeadingError(float x, float y){
  err_ang = (atan2((y-pose_py),(x-pose_px)) - euler[2]);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;
  return err_ang;
}

void sp1::setControlZero(void){
  U_control.twist.angular.z=0;
  U_control.twist.linear.x=0;
  U_control.twist.linear.y=0;
  controlPosition = true;
  controlOrientation = true;
}

void sp1::setDControlZero(void){
  U_control.twist.linear.x=0;
  U_control.twist.linear.y=0;
}

bool sp1::goalReached(void){
  float Gerr;
  Gerr = sqrt(pow(pose_px-Goal.x,2)+pow(pose_py-Goal.y,2));
  if(Gerr<0.5f){
    return true;
  }else{
    return false;
  }
}

void sp1::setGoal(float Gref_x,float Gref_y){
  Goal.x = Gref_x;
  Goal.y = Gref_y;
}