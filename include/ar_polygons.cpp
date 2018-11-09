#include <defs.h>
/*
INS: nc_1, Dnc, name_ch, all_hulls,viewer
OUTS: cloud_names

This method adds a number of polygons if there are new, or removes them from visualization if there are few than before.


Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx    
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/

void add_remove_polygons(int& 										nc_1, 
	int& 															Dnc,
	char* 															name_ch, 
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& 			all_hulls,
	std::vector<std::string>& 										cloud_names){


	if(Dnc>0){
	    //ADD POINT CLOUDS TO THE EXISTING ONES
	    for(int i=0; i<Dnc; i++){
	        sprintf(name_ch,"cloud%d",nc_1+i);
	        cloud_names.push_back(name_ch);
	    }
	}else if(Dnc<0){
		//REMOVE POINT CLOUDS 
	    for(int i=0; i<abs(Dnc);i++){

	        cloud_names.pop_back();
	    }
	}
}
