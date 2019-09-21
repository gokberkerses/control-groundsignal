#ifndef GROUND_STATION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <mavros/mavros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>

//#include <array>
#include <cmath>
#include <queue>
#include <vector>
#include <cstdio>
#include <string>
#include <cstdint>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class GroundStation
{
	private:
		bool gsState;

		bool uavStates[4] ;

		ros::NodeHandle node ;
		ros::Publisher statePublisher ;

		ros::Subscriber stateSubscriber ;
		
	public: 
		// cons - destr
		GroundStation(ros::NodeHandle& _n);
		// getter
		bool isGsReady();
		bool isAllUavReady(); 
		// setter
		void updateStatus();

		void uavCallback (const std_msgs::String::ConstPtr& message) ;

		//
		void publishStatus() ;
} ;

#endif // GROUND_STATION_H
