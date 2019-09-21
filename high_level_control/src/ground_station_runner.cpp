#include "high_level_control/high_level_control_interface.h"
#include "high_level_control/ground_station.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "GroundStation") ; 
	ros::NodeHandle node ;

	GroundStation groundStation(node) ;

	ros::Rate rate(20.0) ;
	while ( ros::ok())
	{
		groundStation.updateStatus() ;
		groundStation.publishStatus() ;
		
		ros::spinOnce() ;
		rate.sleep() ;
	}
}
