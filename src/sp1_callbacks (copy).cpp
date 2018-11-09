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

	ROS_INFO("SP drone");

	cloud_ = nh_.subscribe("/zed/point_cloud/cloud_registered",1, &sp1::pclcall_back_,this);
	subL = it_.subscribe("/zed/depth/depth_registered", 1,&sp1::left_zed_cb, this);
	odome_= nh_.subscribe("/zed/odom",1,&sp1::odom_zed_cb,this);
    pub_hull_to_station = nh_.advertise<sdk::drone_MSG> ("/UAV/hull", 1);


	width = 672;
	height = 376;

    //width = 1280;
    //height = 720;

	izQ4=cv::Mat(height, width, CV_8UC4);
    izQ=cv::Mat(height, width, CV_8UC3);

    cv::namedWindow("VIEW", cv::WINDOW_AUTOSIZE);


    group = 0;

    my_settings.img_size.x = width;
	my_settings.img_size.y = height;
	my_settings.no_segs = atoll(argv[1]);
	my_settings.spixel_size = 16;
	my_settings.coh_weight = atoll(argv[2]);
	my_settings.no_iters = 4;
	my_settings.color_space = gSLICr::RGB; //gSLICr::XYZ; // gSLICr::CIELAB for Lab, or gSLICr::RGB for RGB
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



}

sp1::~sp1(){
	ROS_INFO("TERMINATED");
	destroyAllWindows();
}

void sp1::left_zed_cb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
  izQ4 = cv_ptr->image;
}

void sp1::odom_zed_cb(const nav_msgs::Odometry& data)
{
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


void sp1::pclcall_back_(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg){
	nube = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*nube);

    sp1::sp1Loop();


}


void sp1::sp1Loop(void){

	cvtColor(izQ4,izQ, CV_RGBA2RGB); // Transform the input image to 3 channel type.

	resize(izQ, frame, s); //Resise do desired dimension.

	load_image(frame, in_img); // SP load image method
    
	gSLICr_engine->Process_Frame(in_img); // Perform the SP segmentation

	gSLICr_engine->Draw_Segmentation_Result(out_img); //If you want to see the SP segmentation imshow(out_img) further below.
	
	load_image(out_img, boundry_draw_frame); // Create the SP segmentation on a cv::Mat type ()

	Idx_im = gSLICr_engine->Get_Seg_Res(); // Get the SP indices

	data_ptr = Idx_im->GetData(MEMORYDEVICE_CPU); // Store data on host

	int n_sp = data_ptr[im_sz-1]+1; // Get the total number of SP 

	cv::imshow("VIEW",boundry_draw_frame);

	cv::waitKey(10);

	//------>1st METHOD<----------------------------------------------------------------------------//

	create_connection_ref_mat(conn_com, n_sp);
	

	fillmat.resize  (n_sp);
	col_r.resize    (n_sp);
	col_g.resize    (n_sp);
	col_b.resize    (n_sp);

	//------>2nd METHOD<----------------------------------------------------------------------------//

	find_sp_coordinates(data_ptr, im_sz, fillmat,frame,col_r,col_g,col_b);


	//------>3rd METHOD<----------------------------------------------------------------------------//

    cv::Mat m_ad = cv::Mat(n_sp,n_sp, CV_64FC1,cvScalar(0));
    
    create_adj_mat_mask(data_ptr,m_ad,height,width);

	//------>4th METHOD<----------------------------------------------------------------------------//	    

    extract_colours(kr, kg, kb, col_r, col_g, col_b);

	//------>5th METHOD<----------------------------------------------------------------------------//	    

   	connected_components_list(m_ad,conn_com,cclist, n_sp,group, kr, kg, kb,avg_r,avg_g,avg_b);

   	//------>6th METHOD<----------------------------------------------------------------------------//

    fill_tool(frame2, cclist, avg_r, avg_g, avg_b,fillmat);


    //------>7th METHOD<----------------------------------------------------------------------------//
    		//		|		|		  |       |             |
    fill_pcl_AGL(cclist, fillmat, nube, all_clouds,largest_clouds);
    


    nc=largest_clouds.size();

    //Reset polygons' vectors for each iteration
    all_polygons.clear();
    all_hulls.clear();

    dV1.v_hulls.clear();


     //From 0 to the nth cloud, and clear those not usefull
    for(int ic=0; ic<nc; ic++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_hull (new pcl::PointCloud<pcl::PointXYZRGB>); // Polygon Points

    //------->8th METHOD<----------------------------------------------------------------------------//
    //>->->->->9th METHOD inside<--<------//
    //                   |                |             |          |

        polygons.clear();
        drone_ver1.v_hulls.clear();

        get_polygons(all_clouds, largest_clouds[ic], res_hull, polygons,matAff); //Get the polygon points lying on a plane.
        int hull_size1=res_hull->points.size();
        if(hull_size1>4){ //If the polygon data is big enough, add it, otherwise remove it.
            all_polygons.push_back(polygons);
            all_hulls.push_back(res_hull);
            
            pcl_conversions::fromPCL(polygons,drone_ver1.v_hulls);
            dV1.v_hulls.push_back(drone_ver1);
        }
    }

        //Dnc Difference of the number of clouds between current and previous time.
    nc=all_hulls.size();
	Dnc= nc-nc_1;

    //------->10th METHOD<----------------------------------------------------------------------------//



    add_remove_polygons(nc_1, Dnc,name_ch, all_hulls, cloud_names);

    //------->11th METHOD<---------------------------------------------------------------------------// ROS Publisher inside 11th

    send_hulls(dV1,nc, t_ros, all_hulls, hull_ros_msg, pub_hull_to_station, Pos_ros_msg, Qua_ros_msg,avg_r,avg_g,avg_b,largest_clouds);

    nc_1=nc;

//CLEAR ALL
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
}
