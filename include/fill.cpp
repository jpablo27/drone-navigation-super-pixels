#include <vector>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iomanip>


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

/*
INS: tails, avg_r, avg_g, avg_b, fillmat
OUTS: img_seg2

This method creates a cv::Mat image to visualize the SP based aglomeration using a connected components algorithm
using the averaged colours by all the aglomeration members.

Jose-Pablo Sanchez-Rodriguez	ITESM a01161469@itesm.mx	
Jose Martinez-Carranza			INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez			ITESM aaceves@itesm.mx
*/

void fill_tool(
		Mat&					img_seg2,
		vector<vector<int>>& 	tails,
		vector<double>& 		avg_r,
		vector<double>& 		avg_g,
		vector<double>& 		avg_b,
		vector<vector<int>>&	fillmat

		)
{

	int sp_size,pix, super_pixel;

	for(int AGL=0; AGL < tails.size(); AGL++){

		for(int spi=0; spi<tails[AGL].size();spi++){
			super_pixel=tails[AGL][spi];
			sp_size=fillmat[super_pixel].size();
			for(int i=0; i<sp_size; i++){
				pix=fillmat[super_pixel][i];

				img_seg2.at<Vec3b>(pix)[0] = avg_b[AGL];
				img_seg2.at<Vec3b>(pix)[1] = avg_g[AGL];
				img_seg2.at<Vec3b>(pix)[2] = avg_r[AGL];
			}
		}
	}
}
