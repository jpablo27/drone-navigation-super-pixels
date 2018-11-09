
#include <stdio.h>
#include <vector>
#include <math.h>



#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"


using namespace cv;
using namespace std;

/*

This function creates a connected components list from connected components matrix
and colour comparisson. This function was built recursively in order to avoid group
overlapping and double checking.

Note: 22/DIC/2017
There was a bug about a address problem, this was due to the memory address of the variable
'vector' according to the error thrown.
It was solved declaring the variable as global in this function. The problem appeared in the
'insert' method which was used to concatenate 'temp' and 'g_list' vectors.

Jose-Pablo Sanchez-Rodriguez A01161469@itesm.mx
Jose Martinez-Carranza carranza@inaoep.mx
Alejandro Aceves-Lopez aaceves@itesm.mx

*/
double th=2.5;
vector<int> tempr(0);
int group_t=0;

void cfc(
		int							y,
		int							x,
		cv::Mat&					    ad_mat,
		vector<vector<int>>& 		conexiones,
		int 						sz,
		int&						group_i,
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<double>&				avg_r,
		vector<double>&				avg_g,
		vector<double>&				avg_b,
		vector<vector<int>>& 		g_list,
		bool& t_flag);

void connected_components_listDAB(
		cv::Mat						    ad_mat,
		vector<vector<int>>& 		conexiones,
		vector<vector<int>>& 		g_list,
		int 						sz,
		int& 						group_i,
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<double>&				avg_r,
		vector<double>&				avg_g,
		vector<double>&				avg_b
		)
{

	group_i=0;

	double d_r, d_g, d_b;
	double color_d;


	int x=0,y=0;

	double a_r=0,a_g=0,a_b=0,colcont=0;
	int prom;
	bool t_flag = false;

	for (int i = 0; i < sz; i++)
	{


		if(conexiones[i][1]==0){

			tempr.push_back(i);

			conexiones[i][1]=1;
			conexiones[i][2]=group_i;

			a_r=kr[conexiones[i][0]];
			a_g=kg[conexiones[i][0]];
			a_b=kb[conexiones[i][0]];

			colcont++;

			cfc(i,i,ad_mat,conexiones,sz-i,group_i,kr,kg,kb,avg_r,avg_g,avg_b,g_list,t_flag);

			if(t_flag){

				for(int t_i=0; t_i< tempr.size(); t_i++)
					conexiones[tempr[t_i]][2]=group_t;

				g_list[conexiones[tempr.back()][2]].insert(g_list[conexiones[tempr.back()][2]].end(), tempr.begin(),tempr.end());
				t_flag = false;

			}else{
				g_list.push_back(tempr);
				group_i++;

				prom = a_r/colcont;
				avg_r.push_back(prom);
				prom = a_g/colcont;
				avg_g.push_back(prom);
				prom = a_b/colcont;
				avg_b.push_back(prom);

				colcont=0,a_r=0,a_g=0,a_b=0;
			}
			tempr.clear();
		}
	}
	return;
}


void cfc(
		int							y,
		int							x,
		cv::Mat&					    ad_mat,
		vector<vector<int>>& 		conexiones,
		int 						sz,
		int&							group_i,
	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<double>&				avg_r,
		vector<double>&				avg_g,
		vector<double>&				avg_b,
		vector<vector<int>>& 		g_list,
		bool& t_flag
		){

	double d_r,d_g,d_b,color_d,colcont=0;
	double a_r=0,a_g=0,a_b=0;


	for(int i=1; i<sz; i++){
		d_r=abs(kr[y]-kr[x+i]);
		d_g=abs(kg[y]-kg[x+i]);
		d_b=abs(kb[y]-kb[x+i]);

		color_d=sqrt(0.01*pow(d_r,2) + pow(d_g,2) + pow(d_b,2));

		if((ad_mat.at<double>(y,x+i)==1)&&(color_d<th)){


			if(conexiones[x+i][1]==0){
				conexiones[x+i][1]=1;
				conexiones[x+i][2]=conexiones[y][2];
				tempr.push_back(x+i);

				a_r += kr[x+i];
				a_g += kg[x+i];
				a_b += kb[x+i];

				colcont++;
				cfc(x+i,x+i,ad_mat,conexiones,(sz-i),group_i,kr,kg,kb,avg_r,avg_g,avg_b,g_list,t_flag);

			}else if(conexiones[y][2]!=conexiones[x+i][2]){

				if(!t_flag){
					group_t=conexiones[x+i][2];
					t_flag=true;

				}
			}
		}
	}
}
