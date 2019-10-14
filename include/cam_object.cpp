
#include <pcl/common/common_headers.h>


void draw_camera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ob_camera){

    float size_w = 0.04;
	float size_h = 0.022;
	float p_ = -0.075;
	Eigen::Vector3f cam_0, cam_1, cam_2, cam_3, cam_4;

	cam_0 << 0, 0, 0;
	cam_1 << -p_,  size_w, size_h;
	cam_2 << -p_, -size_w, size_h;
	cam_3 << -p_, -size_w, -size_h;
	cam_4 << -p_,  size_w, -size_h;

	ob_camera->width    = 1;
	ob_camera->height   = 1;
	ob_camera->is_dense = true;
	ob_camera->points.resize (16);

	ob_camera->points[0].x=cam_1(0);
	ob_camera->points[0].y=cam_1(1);
	ob_camera->points[0].z=cam_1(2);
	ob_camera->points[0].r=255;

	ob_camera->points[1].x=cam_2(0);
	ob_camera->points[1].y=cam_2(1);
	ob_camera->points[1].z=cam_2(2);
	ob_camera->points[1].r=255;

	ob_camera->points[2].x=cam_2(0);
	ob_camera->points[2].y=cam_2(1);
	ob_camera->points[2].z=cam_2(2);
	ob_camera->points[2].r=255;

	ob_camera->points[3].x=cam_3(0);
	ob_camera->points[3].y=cam_3(1);
	ob_camera->points[3].z=cam_3(2);
	ob_camera->points[3].r=255;

	ob_camera->points[4].x=cam_3(0);
	ob_camera->points[4].y=cam_3(1);
	ob_camera->points[4].z=cam_3(2);
	ob_camera->points[4].r=255;

	ob_camera->points[5].x=cam_4(0);
	ob_camera->points[5].y=cam_4(1);
	ob_camera->points[5].z=cam_4(2);
	ob_camera->points[5].r=255;

	ob_camera->points[6].x=cam_4(0);
	ob_camera->points[6].y=cam_4(1);
	ob_camera->points[6].z=cam_4(2);
	ob_camera->points[6].r=255;

	ob_camera->points[7].x=cam_1(0);
	ob_camera->points[7].y=cam_1(1);
	ob_camera->points[7].z=cam_1(2);
	ob_camera->points[7].r=255;

	ob_camera->points[8].x=cam_0(0);
	ob_camera->points[8].y=cam_0(1);
	ob_camera->points[8].z=cam_0(2);
	ob_camera->points[8].r=255;

	ob_camera->points[9].x=cam_1(0);
	ob_camera->points[9].y=cam_1(1);
	ob_camera->points[9].z=cam_1(2);
	ob_camera->points[9].r=255;

	ob_camera->points[10].x=cam_0(0);
	ob_camera->points[10].y=cam_0(1);
	ob_camera->points[10].z=cam_0(2);
	ob_camera->points[10].r=255;

	ob_camera->points[11].x=cam_2(0);
	ob_camera->points[11].y=cam_2(1);
	ob_camera->points[11].z=cam_2(2);
	ob_camera->points[11].r=255;

	ob_camera->points[12].x=cam_0(0);
	ob_camera->points[12].y=cam_0(1);
	ob_camera->points[12].z=cam_0(2);
	ob_camera->points[12].r=255;

	ob_camera->points[13].x=cam_3(0);
	ob_camera->points[13].y=cam_3(1);
	ob_camera->points[13].z=cam_3(2);
	ob_camera->points[13].r=255;

	ob_camera->points[14].x=cam_0(0);
	ob_camera->points[14].y=cam_0(1);
	ob_camera->points[14].z=cam_0(2);
	ob_camera->points[14].r=255;

	ob_camera->points[15].x=cam_4(0);
	ob_camera->points[15].y=cam_4(1);
	ob_camera->points[15].z=cam_4(2);
	ob_camera->points[15].r=255;

}
