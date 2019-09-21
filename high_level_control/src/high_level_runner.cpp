#include "high_level_control/high_level_control_interface.h"
#include "high_level_control/ground_station.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

bool _isGroundStationReady = false ;

void gsMessageReceived ( const std_msgs::Bool::ConstPtr& state )
{
	ROS_DEBUG("SimpleRunner: gsMessageReceived is called.") ;
	if (state->data==true  && _isGroundStationReady == false)
		_isGroundStationReady = true ;
}

int main(int argc, char** argv) 
{
	std::string offboard_const = "OFFBOARD";

	std::string hostname(argv[1]);
	ros::init(argc, argv, hostname);
	ros::NodeHandle node;

	ros::Publisher runnerPublisher = node.advertise <std_msgs::String> (
		"/runnerState", 1000
	) ;
	ros::Subscriber gsSubscriber = node.subscribe ("/groundStation", 1000, &gsMessageReceived) ;
	//////
	HighLevelControlInterface agent_control_interface(node, hostname);
	agent_control_interface.setPxParams();
	agent_control_interface.setId(stoi(std::string(argv[2])));

	ros::Rate rate(20.0);
	while (ros::ok() && !agent_control_interface.isConnected()) 
	{
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped p;
	p.pose.position.x = 0.;
	p.pose.position.y = 0.;
	p.pose.position.z = 2.;

	std::vector<geometry_msgs::PoseStamped> wps;
	for (uint i = 0; i < 5; i++) 
	{
		wps.push_back(p);

		if (i % 2 == 0)
			p.pose.position.x += 3.;

		else
			p.pose.position.y += 3.;

		std::cout << p.pose.position.x 
			  << " " << p.pose.position.y 
			  << " " << p.pose.position.z 
			  << std::endl;
	}
	

	ROS_DEBUG("%s: _isGrStRd: %d", hostname.data(), _isGroundStationReady );
	//publish i'm ready message 
	while (ros::ok() && !_isGroundStationReady)
	{
		std_msgs::String message ;
		message.data = hostname.substr(3,1) + std::string("1") ;
		runnerPublisher.publish(message) ;	
		
		ROS_DEBUG("%s: runnerState message is just published.", hostname.data() );
		ROS_DEBUG("%s: _isGrStRd: %d", hostname.data(), _isGroundStationReady );
		ROS_DEBUG("%s: message: %s", hostname.data(), message.data.c_str() );
		ros::spinOnce() ;
		rate.sleep() ;
	}	

	agent_control_interface.loadPlan(wps);

	ros::Time last_request_1(0);
	ros::Time last_request_2(0);

	while (ros::ok()) 
	{
		if (!agent_control_interface.isParamSet())
			agent_control_interface.setPxParams();

		if (agent_control_interface.getMode() != "OFFBOARD" && 
			!agent_control_interface.isOffboardSet() && 
			(ros::Time::now() - last_request_1 > ros::Duration(5.0)) && 
			agent_control_interface.getMode() != "AUTO.LAND") 
		{
			ROS_INFO("Agent %s: Requesting OFFBOARD mode...", agent_control_interface.getHostname().c_str());
			agent_control_interface.setPxMode(offboard_const);
			last_request_1 = ros::Time::now();
		}

		else if (!agent_control_interface.isArmed() 
			&& (ros::Time::now() - last_request_2 > ros::Duration(5.0)) 
			&& agent_control_interface.getMode() != "AUTO.LAND")
		{
			ROS_INFO("Agent %s: Requesting ARM...", agent_control_interface.getHostname().c_str());
			agent_control_interface.armAgent();
			last_request_2 = ros::Time::now();
		}

		agent_control_interface.move();
		if (agent_control_interface.isSetpointReached())
			agent_control_interface.advance();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
