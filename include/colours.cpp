#include <time.h>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <math.h>

using namespace std;
using namespace cv;
/*
INS: col_r, col_g, col_b,
OUT: kr, kg, kb

This method calculates the colour (in average) of each super pixel, and stores it in kr, kg, kb.
This is done by averaging the colour samples in the vectors of vectors col_r, col_g, col_b.

Jose-Pablo Sanchez-Rodriguez	ITESM a01161469@itesm.mx	
Jose Martinez-Carranza			INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez			ITESM aaceves@itesm.mx
*/

void extract_colours(

	    vector<double>& kr,
	    vector<double>& kg,
	    vector<double>& kb,
		vector<vector<double>> col_r,
		vector<vector<double>> col_g,
		vector<vector<double>> col_b
		)
{
	int sz=col_r.size();
	double prom;

	for (int i=0; i<sz; i++){
		double ar=0,ag=0,ab=0;
		double div=col_r[i].size();

		/*There are some cases on which the subsampling in the find_sp_coordinates method 
		causes empty colour samples, hence is not possible to obtain an average*/

		if(div<=0){
			cout<< "[WARNING] Division by zero: "<<div<<endl;
			break;
		}

		for(int j=0; j<div ; j++){
			ab=ab+col_b[i][j];
			ag=ag+col_g[i][j];
			ar=ar+col_r[i][j];
		}

		prom=ab/div;


		kb.push_back(prom);
		prom=ag/div;

		kg.push_back(prom);
		prom=ar/div;

		kr.push_back(prom);
	}
}
