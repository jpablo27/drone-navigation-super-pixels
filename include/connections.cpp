
#include <time.h>
#include <stdio.h>
#include <vector>

using namespace std;

//con_mat , k
//Con_mat es matriz con 3 cols
//Col 0 es indice de super pixel
//Col 1 indica si ya se visitó el pixel o no
//Col 2 indica el grupo al cual pertenece el super pixel

/*
INS: k
OUTS: con_mat

This method initializes a nx3 matrix where n is the number of super pixels.
1st Column is SP index
2nd Column is visit flag
3rd Column is group cell

Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx    
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/

void create_connection_ref_mat(vector<vector<int>>& con_mat, int k){

	vector<int> temp(3);
	

	for (int i=0; i<k; i++){
		con_mat.push_back(temp);
		con_mat[i][0]=i;
		con_mat[i][1]=0;
		con_mat[i][2]=-1;
		
	}

}
