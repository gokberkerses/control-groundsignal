#include "high_level_control/ground_station.h"

void GroundStation::uavCallback (const std_msgs::String::ConstPtr& message) {
	std::string first = message->data.substr(0,1) ;
	std::string second = message->data.substr(1,1) ;
	
	ROS_DEBUG("Ground Station: uavCallback: is called.") ;
	ROS_DEBUG("Ground Station: uavCallback: first=%s, second=%s", first.c_str(), second.c_str() ) ;

	if ( first.compare(std::string("1"))==0 ){
		ROS_DEBUG("Ground Station: uavCallback: inside if, 1x") ;
		if ( second.compare(std::string("1"))==0 )    this->uavStates[0]=true ;
	}

	if ( first.compare(std::string("2"))==0 ){
		ROS_DEBUG("Ground Station: uavCallback: inside if, 2x") ;
		if ( second.compare(std::string("1"))==0 )    this->uavStates[1]=true ;
	}

	if ( first.compare(std::string("3"))==0 ){
		ROS_DEBUG("Ground Station: uavCallback: inside if, 3x") ;
		if ( second.compare(std::string("1"))==0 )    this->uavStates[2]=true ;
	}

	if ( first.compare(std::string("4"))==0 ){
		ROS_DEBUG("Ground Station: uavCallback: inside if, 4x") ;
		if ( second.compare(std::string("1"))==0 )    this->uavStates[3]=true ;
	}
}


GroundStation::GroundStation(ros::NodeHandle& _n): node(_n)
{
	this->gsState = false ;

	this->uavStates[0] = false;
	this->uavStates[1] = false;
	this->uavStates[2] = false;
	this->uavStates[3] = false;

	this->stateSubscriber
	  = this->node.subscribe<std_msgs::String>(
		"/runnerState",
		1000,
		&GroundStation::uavCallback,
		this 
	  ) ;

	this->statePublisher
	  = this->node.advertise<std_msgs::Bool>(
		"/groundStation", 1000
	  ) ;
}


bool GroundStation::isGsReady(){
	return this->gsState ;
}

//-------------------------
bool GroundStation::isAllUavReady(){
	return (this->uavStates[0]==true && this->uavStates[1]==true 
	       && this->uavStates[2]==true && this->uavStates[3]==true) 
		? true : false  ;
}
void GroundStation::updateStatus(){
	if (isAllUavReady())	this->gsState = true ; 
}
//-------------------------

void GroundStation::publishStatus() {
	this->statePublisher.publish(this->gsState) ;
	ROS_DEBUG("Ground Station: %d %d %d %d", uavStates[0], uavStates[1], uavStates[2], uavStates[3]) ;
}
