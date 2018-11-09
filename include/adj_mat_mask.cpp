#include <time.h>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"


/*
INS: klabels, height, width
OUTS: ad_mat

This method fills the adjacence matrix (ad_mat) of size n*n where n is the number of super-pixels. The intersection of
each super-pixel is marked with '1' if both super-pixels are adjacent.

Jose-Pablo Sanchez-Rodriguez	ITESM a01161469@itesm.mx	
Jose Martinez-Carranza			INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez			ITESM aaceves@itesm.mx
*/

int check(int a[]){

	for (int i=8; i>0; i--){
    	if(a[i]!=a[0]){
            return -1;
        }
    }
    return a[0];
}

void fill_mask(int a[], const int*kl,int id_,int width){
/*

Mask:

[LU MU RU]
[LM CC RM]
[LL ML RL]

*/
	a[0]=kl[id_  ];  //Left-Upper Corner          		LU
	a[1]=kl[id_+1];  //Middle-Upper Space				MU
	a[2]=kl[id_+2];  //Right-Upper Corner				RU
	a[3]=kl[width+id_];    //Left-Middle Space			LM
	a[4]=kl[width+id_+1];	//Center Space				CC
	a[5]=kl[width+id_+2];	//Right-Middle Space		RM
	a[6]=kl[2*width+id_];		//Left-Lower Corner		LL
	a[7]=kl[2*width+id_+1];		//Middle-Lower Space	ML
	a[8]=kl[2*width+id_+2];		//Right-Lower Corner	RL
}

void create_adj_mat_mask(const int*klabels,cv::Mat& ad_mat,int height,int width){
	int idx;
	int sp_c,sp_f0,sp_b0;//Super pixel, current (c),forward (f), below (b)
	int n_f0,n_b0;

	int delta=10; // delta greater or equal than 3

	int mask_c[9];
	int mask_f[9];
	int mask_b[9];

	//Adjacency Matrix
	for (int i=0; (i+delta+2)<height ; i+=3){
		for ( int j=0; (j+delta+2)<width; j+=3){

			idx = i*width + j; //Upper-Left corner
			fill_mask(mask_c,klabels,idx,width); //fill the mask with the pixels' marks
			sp_c = check(mask_c);

			if (sp_c<0)
				continue;

			n_f0 =     i*width + (j+delta);
			fill_mask(mask_f,klabels,n_f0,width);
			sp_f0 = check(mask_f);

			if(sp_f0<0)
				continue;

			n_b0 = 	  (i+delta)*width + j;
			fill_mask(mask_b,klabels,n_b0,width);
			sp_b0 = check(mask_b);

			if(sp_b0<0)
				continue;

			//If they are different sp's, then we mark the intersection in t
			//the adjacence matrix
			if((sp_f0 != sp_c)){
				ad_mat.at<double>(sp_f0,sp_c)=1;
				ad_mat.at<double>(sp_c,sp_f0)=1;
			}
		
			if((sp_b0 != sp_c)){
				ad_mat.at<double>(sp_b0,sp_c)=1;
				ad_mat.at<double>(sp_c,sp_b0)=1;
			}
			
		}
	}
}
