#include <map_header.h>
#include <defs.h>
#include <cstdlib>
#include <def_func.h>
#include "st_dr.h"

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


	ROS_INFO("Mapping");


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
  //string filename = "/home/pablo/vid/live.avi"; 
  //string filenam2 = "/home/pablo/vid/noseg.avi";          // name of the output video fil
  //cv::Size soso(672, 376);
  /*DabOut.open(filename,codec,fps,soso,true);
  noseg.open(filenam2,codec,fps,soso,true);

  if(!DabOut.isOpened()){
    std::cout << "No se puede grabar!" << std::endl;
  }else{
    std::cout << "Grabando!" << std::endl;
  }*/

  //Mappoing

  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  	viewer = rgbVis(dummy_cloud);
	  viewer->registerKeyboardCallback(&sp1::keyboard_cbs,*this);
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ob_camera(new pcl::PointCloud<pcl::PointXYZRGB>);
	  draw_camera(ob_camera);

    polygons_cam.resize(1);

    for (int i=0; i<16;i++){
        polygons_cam[0].vertices.push_back(i);
    }

    viewer->addPointCloud<pcl::PointXYZRGB>(ob_camera, "camera");
    viewer->updatePolygonMesh<pcl::PointXYZRGB>(ob_camera, polygons_cam, "camera");
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,0.5,"camera");

    nc_1=0;
    posp.setIdentity();
    matAff.setIdentity();

    mat3.setIdentity();

    map_c_counter=0;
    map_names.resize(0);
    largest_clouds.resize(0);

    coltest = cv::Mat(1,1,CV_8UC3);
    P_map = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

}

sp1::~sp1(){

  /*DabOut.release();
  noseg.release();*/
	std::cout << "TERMINATED" << '\n';

	destroyAllWindows();
}


void sp1::keyboard_cbs(const pcl::visualization::KeyboardEvent &event, void* junk){

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

boost::shared_ptr<pcl::visualization::PCLVisualizer> sp1::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("GROUND STATION VIEWER"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    viewer->setCameraPosition(-6.41523, 0.223382, 2.25379,3.21672, 5.06064, 1.83923,0.090303, -0.094849, 0.991387,0);

    return (viewer);
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


  //noseg.write(frame);


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
    //DabOut.write(frame2);


    //------>7th METHOD<----------------------------------------------------------------------------//
    		//		|		|		  |       |             |
    fill_pcl_AGL(cclist, fillmat, nube, all_clouds,largest_clouds);

    nc=largest_clouds.size();


    //Reset polygons' vectors for each iteration
    all_polygons.clear();
    all_hulls.clear();


     //From 0 to the nth cloud, and clear those not usefull
     centroids.clear();
     num_po.clear();
     errors.clear();
    for(int ic=0; ic<nc; ic++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_hull (new pcl::PointCloud<pcl::PointXYZRGB>); // Polygon Points

    //------->8th METHOD<----------------------------------------------------------------------------//
    //>->->->->9th METHOD inside<--<------//
    //                   |                |             |          |

        polygons.clear();

        sum_error = 0;
        p_count = 0;
        //get_polygons(all_clouds, largest_clouds[ic], res_hull, polygons,matAff,sum_error,p_count,foundObstacle); 
        get_polygons(all_clouds, largest_clouds[ic], res_hull, polygons,matAff,sum_error,pp,p_count,foundObstacle); //Get the polygon points lying on a plane.

        int hull_size1=res_hull->points.size();
        if(hull_size1>3){ //If the polygon data is big enough, add it, otherwise remove it.

          largest_clouds_valid.push_back(largest_clouds[ic]);
          all_polygons.push_back(polygons);
          centroids.push_back(pp);
          num_po.push_back(p_count);
          errors.push_back(sum_error);
          all_hulls.push_back(res_hull);
          yaHay = true;
        }
    }
        //Dnc Difference of the number of clouds between current and previous time.
  nc=all_hulls.size();
	Dnc= nc-nc_1;

    //------->10th METHOD<----------------------------------------------------------------------------//

  add_remove_polygons(nc_1, Dnc,name_ch, all_hulls, cloud_names);

  //------->11th METHOD<---------------------------------------------------------------------------// ROS Publisher inside 11th
  //send_hulls(dV1,nc, t_ros, all_hulls, hull_ros_msg, pub_hull_to_station, Pos_ros_msg, Qua_ros_msg,avg_r,avg_g,avg_b,largest_clouds_valid);

  //Mapping phase

  viewer->spinOnce(1);
  viewer->updatePointCloudPose("camera", matAff);

        if(all_hulls.size()!=largest_clouds.size()){
            //No entrar
            yaHay = false;
        }     
        if(yaHay){

            if(kdfirts){//Inicializacion del mapa
                init_planes(searchPoint,centroids,resolution,P_map,name_ch,viewer,all_hulls,coltest,avg_r, avg_g, avg_b, largest_clouds,r_ply, g_ply, b_ply,all_polygons,errors,num_po);
                // Flag to avoid entering here again
                if(P_map->points.size() > 0){
                    kdfirts = false;
                }
            }else{// Main mapping 
                            //Buscar dentro de este mapa
                kdtree.setInputCloud (P_map);
                map_c_counter = P_map->points.size();
                new_q_planes(searchPoint,centroids,resolution,P_map,name_ch,viewer,all_hulls,coltest,avg_r, avg_g, avg_b, largest_clouds,r_ply, g_ply, b_ply,all_polygons,map_c_counter,kdtree,radius,errors,num_po);

            }

        }

        yaHay = false;

        all_hulls.clear();
        avg_r.clear();
        avg_g.clear();
        avg_b.clear();
        largest_clouds.clear();
        all_polygons.clear();
  //Maping phase until here
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


