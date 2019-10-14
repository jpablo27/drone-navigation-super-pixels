#include <map_header.h>
#include "def_func.h"

/*
This is the first file to be executed.

On object d_n of class sp1 is created with all the ROS instructions inside.

The ROS system is run with the spin() method.

Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/


int main(int argc, char ** argv){

	ros::init (argc, argv, "drone_node");
	sp1 d_n(argc, argv);
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
