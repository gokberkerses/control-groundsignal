#include "high_level_control/high_level_control_interface.h"

HighLevelControlInterface::HighLevelControlInterface(ros::NodeHandle& _n, uint8_t _id) : n(_n), id(_id) 
{
	this->paramSet = false;
	this->emergency = false;
	this->offboard_set = false;
	this->mission_end = false;
	this->mission_start = false;
	this->link_cut = false;
	this->hostname = "uav" + std::to_string(this->id);
	this->master_uri = "";
	this->arm_cmd.request.value = true;
	this->d = 0;
	this->step_done = 0;
	this->step_offset = 0;

	this->initializeRosComponents();
}

HighLevelControlInterface::HighLevelControlInterface(ros::NodeHandle& _n, std::string& _hname) : n(_n), hostname(_hname)
{
	this->paramSet = false;
	this->emergency = false;
	this->offboard_set = false;
	this->mission_end = false;
	this->mission_start = false;
	this->link_cut = false;
	this->master_uri = "";
	this->arm_cmd.request.value = true;
	this->d = 0;
	this->step_done = 0;
	this->step_offset = 0;

	this->initializeRosComponents();
}

HighLevelControlInterface::~HighLevelControlInterface() 
{
	// Destructor
}

void HighLevelControlInterface::setPxParams() 
{
	try 
	{
		mavros_msgs::ParamSet param_vel_manual;
		param_vel_manual.request.param_id = "MPC_VEL_MANUAL";
		param_vel_manual.request.value.real = 3.0;

		mavros_msgs::ParamSet param_xy_cruise;
		param_xy_cruise.request.param_id = "MPC_XY_CRUISE";
		param_xy_cruise.request.value.real = 3.0;

		mavros_msgs::ParamSet param_cruise_90;
		param_cruise_90.request.param_id = "MPC_CRUISE_90";
		param_cruise_90.request.value.real = 2.0;

		mavros_msgs::ParamSet param_xy_vel_max;
		param_xy_vel_max.request.param_id = "MPC_XY_VEL_MAX";
		param_xy_vel_max.request.value.real = 1.0;

		mavros_msgs::ParamSet param_z_vel_max;
		param_z_vel_max.request.param_id = "MPC_Z_VEL_MAX_UP";
		param_z_vel_max.request.value.real = 1.0;

		if(set_max_vel_client.call(param_vel_manual))
		{
			ROS_INFO("UAV1 MPC_VEL_MANUAL parameter change is called!");
			sleep(2);
		}

		if(set_max_vel_client.call(param_xy_cruise))
		{
			ROS_INFO("UAV1 MPC_XY_CRUISEL parameter change is called!");
			sleep(2);
		}

		if(set_max_vel_client.call(param_cruise_90))
		{
			ROS_INFO("UAV1 MPC_CRUISE_90 parameter change is called!");
			sleep(2);
		}

		if(set_max_vel_client.call(param_xy_vel_max))
		{
			ROS_INFO("UAV1 MPC_XY_VEL_MAX parameter change is called!");
			sleep(2);
		}

		if(set_max_vel_client.call(param_z_vel_max))
		{
			ROS_INFO("UAV1 MPC_Z_VEL_MAX_UP parameter change is called!");
			sleep(2);
		}

		this->paramSet = true;
	}
	catch (std::exception& e) 
	{
		ROS_INFO("Error while (Agent: %s) HighLevelControlInterface::setPxParams: \n%s", this->hostname.c_str(), e.what());
		return;
	}
}

void HighLevelControlInterface::armAgent() 
{
	if (this->arm_client.call(this->arm_cmd) && this->arm_cmd.response.success) 
		ROS_INFO("Agent: %s - Armed.", this->hostname.c_str());

	else
		ROS_INFO("Agent: %s - Arm FAILED!.", this->hostname.c_str());
}

void HighLevelControlInterface::setPxMode(std::string& m) 
{
	std::string prev_px_mode = this->mav_mode.request.custom_mode;
	this->mav_mode.request.custom_mode = m;

	if (this->mode_set_client.call(this->mav_mode) && this->mav_mode.response.mode_sent) 
	{
		ROS_INFO("Agent: %s - Px Mode changed to %s.", this->hostname.c_str(), m.c_str());

		//if (m == "OFFBOARD")
		//	this->offboard_set = true;
	}

	else if (this->current_state.mode != "OFFBOARD")
	{
		ROS_INFO("Agent: %s - Px Mode change to %s FAILED!", this->hostname.c_str(), m.c_str());
		this->mav_mode.request.custom_mode = prev_px_mode;
	}
}

void HighLevelControlInterface::initializeRosComponents() 
{
	this->local_pos_sub 
	  = this->n.subscribe<nav_msgs::Odometry>(
	  	"/" + this->hostname + "/mavros/global_position/local", 
		10, 
		&HighLevelControlInterface::local_pos_callback, 
		this
	  );
	this->mav_state_sub 
	  = this->n.subscribe<mavros_msgs::State>(
	  	"/" + this->hostname + "/mavros/state", 
		10, 
		&HighLevelControlInterface::mav_state_callback, 
		this
	  );
	this->setpoint_pub
	  = this->n.advertise<geometry_msgs::PoseStamped>(
	  	"/" + this->hostname + "/mavros/setpoint_position/local", 
		10
	  );
	this->arm_client 
      	  = this->n.serviceClient<mavros_msgs::CommandBool>(
	  	"/" + this->hostname + "/mavros/cmd/arming"
	  );
	this->mode_set_client 
	  = this->n.serviceClient<mavros_msgs::SetMode>(
	  	"/" + this->hostname + "/mavros/set_mode"
	  );
	this->set_max_vel_client 
	  = this->n.serviceClient<mavros_msgs::ParamSet>(
	  	"/" + this->hostname + "/mavros/param/set"
	  );
	// this->mav_action_sub = this->n.subscribe<>
}

uint8_t HighLevelControlInterface::getId()
{
	return this->id;
}

std::string HighLevelControlInterface::getMode() 
{
	return this->current_state.mode;
}

std::string HighLevelControlInterface::getHostname()
{
	return this->hostname;
}

std::string HighLevelControlInterface::getMasterUri()
{
	return this->master_uri;
}

bool HighLevelControlInterface::isArmed() 
{
	return this->current_state.armed;
}

bool HighLevelControlInterface::isParamSet()
{
	return this->paramSet;
}

bool HighLevelControlInterface::isOffboardSet() 
{
	return this->offboard_set;
}

bool HighLevelControlInterface::isMissionStarted() 
{
	return this->mission_start;
}

bool HighLevelControlInterface::isMissionEnded() 
{
	return this->mission_end;
}

bool HighLevelControlInterface::isConnected() 
{
	return this->current_state.connected;
}

bool HighLevelControlInterface::isSetpointReached() 
{
	this->d = sqrt(pow(this->local_pos.pose.pose.position.x - this->setpoint.pose.position.x, 2) + 
				pow(this->local_pos.pose.pose.position.y - this->setpoint.pose.position.y, 2) + 
				pow(this->local_pos.pose.pose.position.z - this->setpoint.pose.position.z, 2));

	//ROS_INFO("Agent %s: Distance to the next setpoint: %.5f m.", this->hostname.c_str(), this->d);
	return this->d < 1.5;
}

void HighLevelControlInterface::advance() 
{
	this->step_done++;

	if (this->step_done < this->plan.size())
		this->setSetpoint(this->plan[this->step_done]);
}

void HighLevelControlInterface::setId(uint8_t _id)
{
	this->id = _id;
}

void HighLevelControlInterface::setHostname(std::string& _hname)
{
	this->hostname = _hname;
}

void HighLevelControlInterface::setMasterUri(std::string& _m_uri)
{
	this->master_uri = _m_uri;
}

void HighLevelControlInterface::setSetpoint(geometry_msgs::PoseStamped& p) 
{
	this->setpoint = p;
	// ROS_INFO("Agent: %s - Next setpoint: (%.3f, %.3f, %.3f)", 
	// 	this->hostname.c_str(), 
	// 	this->setpoint.pose.position.x, 
	// 	this->setpoint.pose.position.y, 
	// 	this->setpoint.pose.position.z);
}

void HighLevelControlInterface::loadPlan(std::vector<geometry_msgs::PoseStamped>& p)
{
	this->plan = p;
	this->step_done = 0;
	this->setpoint = this->plan[this->step_offset + this->step_done];

	ROS_INFO("Agent: %s - Plan:", this->hostname.c_str());
	uint s = p.size();
	for (uint i = 0; i < s; i++)
	{
		ROS_INFO("Waypoint %d: (%.3f, %.3f, %.3f)", 
			i, 
			this->plan[i].pose.position.x, 
			this->plan[i].pose.position.y, 
			this->plan[i].pose.position.z);
	}

	ROS_INFO("Agent: %s - Plan loaded. Next setpoint: (%.3f, %.3f, %.3f), global id: %d", 
		this->hostname.c_str(), 
		this->setpoint.pose.position.x, 
		this->setpoint.pose.position.y, 
		this->setpoint.pose.position.z, 
		this->step_offset + this->step_done);

	this->step_offset += this->plan.size();
}

void HighLevelControlInterface::move() 
{
	this->setpoint_pub.publish(this->setpoint);
	// ROS_INFO("Agent: %s - Published setpoint: (%.3f, %.3f, %.3f)", 
	// 	this->hostname.c_str(), 
	// 	this->setpoint.pose.position.x, 
	// 	this->setpoint.pose.position.y, 
	// 	this->setpoint.pose.position.z);
}

void HighLevelControlInterface::local_pos_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	this->local_pos = *msg;
}

void HighLevelControlInterface::mav_state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	this->current_state = *msg;
}

//void HighLevelControlInterface::mav_action_callback(const high_level_control::MavAction::ConstPtr& msg) {}
