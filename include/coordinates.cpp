#include <stdio.h>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
/*
INS:klabels, im_sz, img
OUTS: fillmat, col_r_g_b

This method saves the relation of super-pixels with its corresponding pixels. fillmat vector contains on its first
field the super-pixel index and the pixel index on the second field. E.g. fillmat[super-pixel][pixels on such super-
pixel]

Jose-Pablo Sanchez-Rodriguez	ITESM a01161469@itesm.mx	
Jose Martinez-Carranza			INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez			ITESM aaceves@itesm.mx
*/
void find_sp_coordinates(
		const int* &klabels,
		int& im_sz,
		vector<vector<int>>& fillmat, 
		Mat img,
		vector<vector<double>>& col_r,
		vector<vector<double>>& col_g,
		vector<vector<double>>& col_b){

	bool flag1=false;

	for(int i=0; i<im_sz; i++){

		// klabels[i] is the superpixel number on which the pixel 'i' is located.
		//fillmat[klabels[i]] is a vector ennumerating all the pixels in the #curr super-pixel accordingly.
		fillmat[klabels[i]].push_back(i);
		
		//col_b _g _r contains samples (e.g. 1 for each 8) of the rgb values of each super-pixel #curr
		if((i%8) == 0){

			//if((i>0)&&double(img.at<Vec3b>(i)[0]))

			col_b[klabels[i]].push_back(double(img.at<Vec3b>(i)[0]));
			col_g[klabels[i]].push_back(double(img.at<Vec3b>(i)[1]));
			col_r[klabels[i]].push_back(double(img.at<Vec3b>(i)[2]));
		}
	}
}


