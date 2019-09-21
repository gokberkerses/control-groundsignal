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
#include <nav_msgs/Odometry.h>

mavros_msgs::State current_state_1;
void state_cb_1(const mavros_msgs::State::ConstPtr& msg) {
    current_state_1 = *msg;
}

mavros_msgs::State current_state_2;
void state_cb_2(const mavros_msgs::State::ConstPtr& msg) {
    current_state_2 = *msg;
}

nav_msgs::Odometry local_pos_1;
void local_cb_1(const nav_msgs::Odometry::ConstPtr& msg_l) {
    local_pos_1 = *msg_l;
}

nav_msgs::Odometry local_pos_2;
void local_cb_2(const nav_msgs::Odometry::ConstPtr& msg_l) {
    local_pos_2 = *msg_l;
}

/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
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


/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
	bool mission_end_1 = false;
    bool mission_end_2 = false;

    bool set_offboard_1 = false;
    bool set_offboard_2 = false;

    bool took_off_1 = false;
    bool took_off_called_1 = false;

    bool took_off_2 = false;
    bool took_off_called_2 = false;


    ros::init(argc, argv, "offb_main_2");
    ros::NodeHandle nh;

    ros::Subscriber state_sub_1 = nh.subscribe<mavros_msgs::State>
                                ("/uav1/mavros/state", 10, state_cb_1);
    ros::Publisher local_pos_pub_1 = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_1 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_1 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/uav1/mavros/set_mode");
    ros::Subscriber local_sub_1 = nh.subscribe<nav_msgs::Odometry>
                                ("/uav1/mavros/global_position/local", 10, local_cb_1);                                         

    ros::Subscriber state_sub_2 = nh.subscribe<mavros_msgs::State>
                                ("/uav2/mavros/state", 10, state_cb_2);
    ros::Publisher local_pos_pub_2 = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_2 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_2 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/uav2/mavros/set_mode");
    ros::Subscriber local_sub_2 = nh.subscribe<nav_msgs::Odometry>
                                ("/uav2/mavros/global_position/local", 10, local_cb_2);                                          

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(20.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state_1.connected && current_state_2.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_1;
    pose_1.pose.position.x = 0;
    pose_1.pose.position.y = 0;
    pose_1.pose.position.z = 2;

    geometry_msgs::PoseStamped pose_2;
    pose_2.pose.position.x = 0;
    pose_2.pose.position.y = 0;
    pose_2.pose.position.z = 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode takeoff_mode_1;
    takeoff_mode_1.request.custom_mode = "AUTO.TAKEOFF";    

    mavros_msgs::SetMode takeoff_mode_2;
    takeoff_mode_2.request.custom_mode = "AUTO.TAKEOFF";  

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request_1(0);
    ros::Time last_request_2(0);

    while (ros::ok()) {
        if (current_state_1.mode != "OFFBOARD" && !set_offboard_1 &&
                (ros::Time::now() - last_request_1 > ros::Duration(5.0)) && current_state_1.mode != "AUTO.LAND" && took_off_1 == true) {
            if( set_mode_client_1.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled for UAV1");
            }
            last_request_1 = ros::Time::now();
        } else {

            if (!current_state_1.armed &&
                    (ros::Time::now() - last_request_1 > ros::Duration(5.0)) && current_state_1.mode != "AUTO.LAND") {
                if( arming_client_1.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("UAV1 is armed");
                }
                last_request_1 = ros::Time::now();
            }
        }

        if (current_state_2.mode != "OFFBOARD" && !set_offboard_2 &&
                (ros::Time::now() - last_request_2 > ros::Duration(5.0)) && current_state_2.mode != "AUTO.LAND" && took_off_1 == true) {
            if( set_mode_client_2.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled for UAV2");
            }
            last_request_2 = ros::Time::now();
        } else {

            if (!current_state_2.armed &&
                    (ros::Time::now() - last_request_2 > ros::Duration(5.0)) && current_state_2.mode != "AUTO.LAND" ) {
                if( arming_client_2.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("UAV2 is armed");
                }
                last_request_2 = ros::Time::now();
            }
        }

        if(current_state_1.mode == "OFFBOARD")
        {
            set_offboard_1 = true;
            ROS_INFO("MODE CHANGED TO OFFBOARD!!!");
        }

        if(current_state_2.mode == "OFFBOARD")
        {
            set_offboard_2 = true;
            ROS_INFO("MODE CHANGED TO OFFBOARD!!!");
        }

        if(took_off_called_1 == false)
        {

            if( set_mode_client_1.call(takeoff_mode_1) &&
                    takeoff_mode_1.response.mode_sent) {
                ROS_INFO("Takeoff sent to UAV1");
            }

            ROS_INFO("Take off of UAV1 is initiated!!!");
            took_off_called_1 = true;
        }

        if(took_off_called_2 == false)
        {

            if( set_mode_client_2.call(takeoff_mode_2) &&
                    takeoff_mode_2.response.mode_sent) {
                ROS_INFO("Takeoff sent to UAV 2");
            }

            ROS_INFO("Take off of UAV2 is initiated!!!");
            took_off_called_2 = true;
        }                

        int c = getch();
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                pose_1.pose.position.x += 1;
                pose_2.pose.position.x += 1;
                break;
            case 66:    // key down
                pose_1.pose.position.x += -1;
                pose_2.pose.position.x += -1;
                break;
            case 67:    // key right
                pose_1.pose.position.y += 1;
                pose_2.pose.position.y += 1;
                break;
            case 68:    // key left
                pose_1.pose.position.y += -1;
                pose_2.pose.position.y += -1;
                break;
            case 102://f
                return 0;
                break;
            case 115: //s
			    mavros_msgs::SetMode land_set_mode;
			    land_set_mode.request.custom_mode = "AUTO.LAND";

				mission_end_1 = true;
                mission_end_2 = true;

				if( set_mode_client_1.call(land_set_mode) && land_set_mode.response.mode_sent) 
				{
				    ROS_INFO("Land send to UAV1");
				}

                if( set_mode_client_2.call(land_set_mode) && land_set_mode.response.mode_sent) 
                {
                    ROS_INFO("Land send to UAV2");
                }
        }

    }

        if(abs(local_pos_1.pose.pose.position.z-2.5)<0.5)
        {
            took_off_1 =true;

        }  

        if(abs(local_pos_2.pose.pose.position.z-2.5)<0.5)
        {
            took_off_2 =true;

        }  

        if(mission_end_1 == false && took_off_1 == true)
        {
	        local_pos_pub_1.publish(pose_1);
	        ROS_INFO("UAV1 setpoint: %.1f, %.1f, %.1f", pose_1.pose.position.x, pose_1.pose.position.y, pose_1.pose.position.z);
        }

        if(mission_end_2 == false && took_off_2 == true)
        {
            local_pos_pub_2.publish(pose_2);
            ROS_INFO("UAV2 setpoint: %.1f, %.1f, %.1f", pose_2.pose.position.x, pose_2.pose.position.y, pose_2.pose.position.z);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}