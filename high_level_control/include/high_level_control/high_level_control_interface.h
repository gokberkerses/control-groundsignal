#ifndef HIGH_LEVEL_CONTROL_INTERFACE_H
#define HIGH_LEVEL_CONTROL_INTERFACE_H

#include <ros/ros.h>
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

class HighLevelControlInterface 
{
	private:
		uint8_t id;
		bool paramSet;
		bool emergency;
		bool link_cut;
		bool offboard_set;
		bool mission_end;
		bool mission_start;
		double d;
		uint step_done;
		uint step_offset;

		std::string hostname;
		std::string master_uri;

		mavros_msgs::State current_state;
		mavros_msgs::SetMode mav_mode;
		mavros_msgs::CommandBool arm_cmd;
		nav_msgs::Odometry local_pos;
		geometry_msgs::PoseStamped setpoint;
		std::vector<geometry_msgs::PoseStamped> plan;

		ros::NodeHandle n;

		ros::Publisher setpoint_pub;

		ros::ServiceClient arm_client;
		ros::ServiceClient mode_set_client;
		ros::ServiceClient set_max_vel_client;

		ros::Subscriber mav_state_sub;
		ros::Subscriber local_pos_sub;

		ros::Subscriber mav_action_sub;

	public:
		// Constructors & Destructor
		HighLevelControlInterface(ros::NodeHandle& _n, uint8_t _id);
		HighLevelControlInterface(ros::NodeHandle& _n, std::string& _hname);
		~HighLevelControlInterface();

		// Pixhawk & ROS initializers
		void setPxParams();
		void armAgent();
		void setPxMode(std::string& m);
		void initializeRosComponents();

		// Getters
		uint8_t getId();
		std::string getMode();
		std::string getHostname();
		std::string getMasterUri();
		bool isArmed();
		bool isParamSet();
		bool isOffboardSet();
		bool isMissionStarted();
		bool isMissionEnded();
		bool isConnected();
		bool isSetpointReached();

		// Setters
		void advance();
		//void resume();
		void setId(uint8_t _id);
		void setHostname(std::string& _hname);
		void setMasterUri(std::string& _m_uri);
		void setSetpoint(geometry_msgs::PoseStamped& p);
		void loadPlan(std::vector<geometry_msgs::PoseStamped>& p);

		// Primitive Actions
		//void takeoff();
		//void land();
		void move();

		// ROS Topic Callbacks & Service Handlers
		void local_pos_callback(const nav_msgs::Odometry::ConstPtr& msg);
		void mav_state_callback(const mavros_msgs::State::ConstPtr& msg);
		//void mav_action_callback(const high_level_control::MavAction::ConstPtr& msg);
};

#endif // HIGH_LEVEL_CONTROL_INTERFACE_H
