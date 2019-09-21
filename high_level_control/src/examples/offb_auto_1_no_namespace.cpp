#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <mavros/mavros.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <nav_msgs/Odometry.h>
#include<unistd.h>

bool parameter_set_1 = false;

int wp_1 = 0;

mavros_msgs::State current_state_1;
void state_cb_1(const mavros_msgs::State::ConstPtr& msg) {
    current_state_1 = *msg;
}

nav_msgs::Odometry local_pos_1;
void local_cb_1(const nav_msgs::Odometry::ConstPtr& msg_l) {
    local_pos_1 = *msg_l;
}

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

void set_params(ros::ServiceClient set_max_vel_client_1)//, ros::ServiceClient set_max_vel_client_2)
{

    mavros_msgs::ParamSet param_vel_manual_1;
    param_vel_manual_1.request.param_id = "MPC_VEL_MANUAL";
    param_vel_manual_1.request.value.real = 4.0;	

    mavros_msgs::ParamSet param_xy_cruise_1;
    param_xy_cruise_1.request.param_id = "MPC_XY_CRUISE";
    param_xy_cruise_1.request.value.real = 3.0;

    mavros_msgs::ParamSet param_xy_vel_max_1;
    param_xy_vel_max_1.request.param_id = "MPC_XY_VEL_MAX";
    param_xy_vel_max_1.request.value.real = 4.0;

    if(set_max_vel_client_1.call(param_vel_manual_1))
    {
    	ROS_INFO("UAV1 MPC_VEL_MANUAL parameter change is called!");
    	sleep(2);
    }

    if(set_max_vel_client_1.call(param_xy_cruise_1))
    {
    	ROS_INFO("UAV1 MPC_XY_CRUISEL parameter change is called!");
    	sleep(2);
    }

    if(set_max_vel_client_1.call(param_xy_vel_max_1))
    {
    	ROS_INFO("UAV1 MPC_XY_VEL_MAX parameter change is called!");
    	sleep(2);
    }

    if(param_vel_manual_1.response.success && param_xy_cruise_1.response.success 
    	&& param_xy_vel_max_1.response.success)
    {
    	parameter_set_1 = true;
    	ROS_INFO("UAV1 parameters are set!");
    	sleep(2);
    }

}

int main(int argc, char **argv)
{
	bool mission_end_1 = false;

	bool mission_start_1 = false;

    bool set_offboard_1 = false;

    ros::init(argc, argv, "offb_auto");
    ros::NodeHandle nh;

    ros::Subscriber state_sub_1 = nh.subscribe<mavros_msgs::State>
                                ("/mavros/state", 10, state_cb_1);
    ros::Publisher local_pos_pub_1 = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_1 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_1 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");
    ros::ServiceClient set_max_vel_client_1 = nh.serviceClient<mavros_msgs::ParamSet>
                                         ("/mavros/param/set");
    ros::Subscriber local_sub_1 = nh.subscribe<nav_msgs::Odometry>
                                ("/mavros/global_position/local", 10, local_cb_1);                                                                  

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(20.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state_1.connected){// && current_state_2.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_1;
    pose_1.pose.position.x = 0;
    pose_1.pose.position.y = 0;
    pose_1.pose.position.z = 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request_1(0);

    while (ros::ok()) {

    	if(!parameter_set_1)
    		set_params(set_max_vel_client_1);


        if (current_state_1.mode != "OFFBOARD" && !set_offboard_1 &&
                (ros::Time::now() - last_request_1 > ros::Duration(5.0)) && current_state_1.mode != "AUTO.LAND") {
            if( set_mode_client_1.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled for UAV1");
            }
            last_request_1 = ros::Time::now();
        } else {

            if (!current_state_1.armed &&
                    (ros::Time::now() - last_request_1 > ros::Duration(5.0)) && current_state_1.mode != "AUTO.LAND" ) {
                if( arming_client_1.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("UAV1 is armed");
                }
                last_request_1 = ros::Time::now();
            }
        }

        if(current_state_1.mode == "OFFBOARD")
        {
            set_offboard_1 = true;
            ROS_INFO("MODE CHANGED TO OFFBOARD!!!");
        }

        int c = getch();
        if (c != EOF && c == 115) {

		    mavros_msgs::SetMode land_set_mode;
		    land_set_mode.request.custom_mode = "AUTO.LAND";

			mission_end_1 = true;

			if( set_mode_client_1.call(land_set_mode) && land_set_mode.response.mode_sent) 
			{
			    ROS_INFO("Land send to UAV1");
			}
    }

    	else if(c != EOF && c == 116)
    	{
			mission_end_1 = true;

			mission_start_1 = false;

		    mavros_msgs::SetMode land_set_mode;
		    land_set_mode.request.custom_mode = "AUTO.LAND";

			if( set_mode_client_1.call(land_set_mode) && land_set_mode.response.mode_sent) 
			{
			    ROS_INFO("Land send to UAV1");
			}
    	}

    	else if(c != EOF && c == 103)
    	{
			mission_start_1 = true;
    	}

        switch (wp_1) {
        case 0:    // key up
            pose_1.pose.position.x = 0;
            pose_1.pose.position.y = 0;
            pose_1.pose.position.z = 2;
            break;    
        case 1:    // key up
            pose_1.pose.position.x = 0;
            pose_1.pose.position.y = 10;
            pose_1.pose.position.z = 2;
            break;
        case 2:    // key down
            pose_1.pose.position.x = 10;
            pose_1.pose.position.y = 10;
            pose_1.pose.position.z = 2;
            break;
        case 3:    // key right
            pose_1.pose.position.x = 10;
            pose_1.pose.position.y = 0;
            pose_1.pose.position.z = 2;
            break;
        case 4:    // key left
            pose_1.pose.position.x = 0;
            pose_1.pose.position.y = 0;
            pose_1.pose.position.z = 2;
            break;
        }

        if(sqrt(pow(local_pos_1.pose.pose.position.x-pose_1.pose.position.x,2)+pow(local_pos_1.pose.pose.position.y-pose_1.pose.position.y,2)+pow(local_pos_1.pose.pose.position.z-pose_1.pose.position.z,2))<1.0)
        {
        	if(mission_start_1)
            	wp_1 = wp_1+1;

        }   

        if(mission_end_1 == false)
        {
	        local_pos_pub_1.publish(pose_1);
	        // ROS_INFO("UAV1 setpoint: %.1f, %.1f, %.1f", pose_1.pose.position.x, pose_1.pose.position.y, pose_1.pose.position.z);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}