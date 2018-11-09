#include <sp1_header.h>
#include <RRT.h>

/*
This is the first file to be executed.

On object d_n of class sp1 is created with all the ROS instructions inside.

The ROS system is run with the spin() method.

Jose-Pablo Sanchez-Rodriguez    ITESM a01161469@itesm.mx
Jose Martinez-Carranza          INAOE carranza@inaoep.mx
Alejandro Aceves-Lopez          ITESM aaceves@itesm.mx
*/

bool f_Obstacle=false;
bool poseflag_=false;
int wp_th_num = 5;
bool checkingFlag = false;
bool onlyRotate=false;
bool canDisplace=false;
geometry_msgs::Point wpPosition;
geometry_msgs::Point init;

float err_distTH = 0.3;
float err_angTH = 5;
float err_ang;

int Tobs_count=0;
int Fobs_count=0;

float ref_x;
float ref_y;

int main(int argc, char ** argv){



	/*if(argc == 3){
	  ref_x = std::atof(argv[1]);
	  ref_y = std::atof(argv[2]);
	}else{

	    ROS_ERROR("GIMME X Y GOAL COORDINATES");
	    return -1;
	}*/

	float wp_i;

	ros::init (argc, argv, "drone_node");
	sp1 d_n(argc, argv);
	//d_n.setGoal(ref_x,ref_y);
	ros::Rate loop_rate(20);

// Wait For MAVROS to publish the Odom topic/*

	while((!poseflag_)&&(!d_n.getFlagsState())){
		poseflag_=d_n.getPoseFlag();
		ros::spinOnce();
		loop_rate.sleep();
	}

	init = d_n.getPosition();

	RRT tree(init);

	while (ros::ok()) {

		if (d_n.getFlagsState()) {
			f_Obstacle = false;
			f_Obstacle = d_n.sp1Loop(); // Loop principal de super pixeles Regresar true si hay obstáculos, false si NO hay obstáculos.

			if(f_Obstacle){
				std::cout << "SI hay obstáculo" << std::endl;
				Tobs_count++;
			}else{
				std::cout << "NO hay obstáculo" << std::endl;
			}
		}
		

		//SCOPE2
		if(d_n.getWpControlState()){//Not moving

			if(!checkingFlag){
				if(tree.getNumberOfTrials()==0){
					//Orientate towards Goal
					wp_i = d_n.getGoalYaw();
					//Rotate -------------------->
				}else if(tree.getNumberOfTrials()>4){
					//Move towards parent
					wp_i = tree.getParentGlobalYaw();

					if(wp_i!=-100){
						//ROTATE & Displacement ---------------------->
					}else{//If the init node is full, end the program
						ros::shutdown();
						break;
					}

				}else{
					do{
						wp_i = d_n.getProposedWp();

					}while(!tree.isValidWp(wp_i)); //Loop until Valid wp
					//Rotate----------------------------->
				}
				onlyRotate = true;
				d_n.setOnlyRotateT();
				checkingFlag = true;
				wpPosition = d_n.getNewSetPoint(wp_i);//Angle to Rectangular

			}else{

				if(f_Obstacle){
					//add obstacle
					tree.AddObstacleChild(wp_i);
					checkingFlag = false;
					canDisplace = false;
					d_n.setOnlyDisplaceF();
					onlyRotate = true;
					d_n.setOnlyRotateT();
				}else{
					tree.AddNodeToCurrent(wp_i,wpPosition);//Add and move
					canDisplace = true;//Displacement 
					d_n.setOnlyDisplaceT();
					onlyRotate = false;// DONT YAW --------------------------->
					d_n.setOnlyRotateF();
				}

			}

		//IF NOT MOVING
		}else{

			//IF MOVING
			if(d_n.getDistanceError(wpPosition.x,wpPosition.y)>err_distTH){
				d_n.YawControl(wpPosition.x,wpPosition.y);
				err_ang = d_n.getHeadingError(wpPosition.x,wpPosition.y);
				if((err_ang<err_angTH)&&(err_ang>-err_angTH)){

					if(onlyRotate){
						d_n.setControlZero();
						onlyRotate = false;
					}else{

						if(canDisplace){
							d_n.PositionControl(wpPosition.x,wpPosition.y);
						}
					}
				}else{
					d_n.setDControlZero();
				}
			}else{
				d_n.setControlZero();
				canDisplace = false;
				d_n.setOnlyDisplaceF();
				onlyRotate = false;
				d_n.setOnlyRotateF();
				checkingFlag = false;
			}
		}

		if(d_n.goalReached()){
			d_n.setControlZero();
			d_n.publishMAVROS();
			ros::spinOnce();
			loop_rate.sleep();
			std::cout << "just REACHED the GOAL!" << std::endl;
			return 1;
		}
		
		//Publishing Method
		d_n.publishMAVROS();

		//Estas dos líneas de preferencia que se ejecuten al último
		ros::spinOnce();
		loop_rate.sleep();
	//While loop	  
	}
	return 0;
}
