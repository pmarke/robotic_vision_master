// #include <ros/ros.h>
#include "frontend.h"

int main(int argc, char** argv){
	// start node
	ros::init(argc, argv, "frontend_node");


	robotic_vision::Frontend frontend;


	ros::spin();
	return 0;
}