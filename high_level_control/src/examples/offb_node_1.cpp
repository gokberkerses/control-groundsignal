// /*/*/**
//  * @file offb_main.cpp
//  * @author Julian Oes <julian@oes.ch>
//  * @license BSD 3-clause
//  *
//  * @brief ROS node to do offboard control of PX4 through MAVROS.
//  *
//  * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
//  */


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


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

nav_msgs::Odometry local_pos_1;
void local_cb_1(const nav_msgs::Odometry::ConstPtr& msg_l) {
    local_pos_1 = *msg_l;
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
	bool mission_end = false;
    bool set_offboard = false;
    bool took_off = false;
    bool took_off_called = false;

    ros::init(argc, argv, "offb_main");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("/uav1/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/uav1/mavros/set_mode");
    ros::Subscriber local_sub_1 = nh.subscribe<nav_msgs::Odometry>
                                ("/uav1/mavros/global_position/local", 10, local_cb_1); 

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(20.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode takeoff_mode;
    takeoff_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request(0);

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && !set_offboard && 
                (ros::Time::now() - last_request > ros::Duration(5.0)) && current_state.mode != "AUTO.LAND" && took_off == true) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

        else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)) && current_state.mode != "AUTO.LAND" ) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(current_state.mode == "OFFBOARD")
        {
            set_offboard = true;
            ROS_INFO("MODE CHANGED TO OFFBOARD!!!");
        }

        if(took_off_called == false)
        {

            if( set_mode_client.call(takeoff_mode) &&
                    takeoff_mode.response.mode_sent) {
                ROS_INFO("Takeoff sent");
            }

            ROS_INFO("Take off initiated!!!");
            took_off_called = true;
        }



        int c = getch();
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                pose.pose.position.x += 1;
                break;
            case 66:    // key down
                pose.pose.position.x += -1;
                break;
            case 67:    // key right
                pose.pose.position.y += 1;
                break;
            case 68:    // key left
                pose.pose.position.y += -1;
                break;
            case 63:
                return 0;
                break;
            case 115:
			    mavros_msgs::SetMode land_set_mode;
			    land_set_mode.request.custom_mode = "AUTO.LAND";
				mission_end = true;
				if( set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) 
				{
				ROS_INFO("Land send");
				}
        }

    }

        if(abs(local_pos_1.pose.pose.position.z-2.5)<0.5)
        {
        	took_off =true;

        }   


        if(mission_end == false && took_off == true)
        {
	        local_pos_pub.publish(pose);
	        ROS_INFO("setpoint: %.1f, %.1f, %.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */


// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <geometry_msgs/TwistWithCovariance.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <cstdio>
// #include <unistd.h>
// #include <termios.h>
// #include <fcntl.h>
// #include <math.h>
// #include <nav_msgs/Odometry.h>

// int wp = 0;


// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg) {
//     current_state = *msg;
// }

// nav_msgs::Odometry local_pos;
// void local_cb(const nav_msgs::Odometry::ConstPtr& msg_l) {
//     local_pos = *msg_l;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "offb_main");
//     ros::NodeHandle nh;

//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//                                 ("mavros/state", 10, state_cb);
//     ros::Subscriber local_sub = nh.subscribe<nav_msgs::Odometry>
//                                 ("/mavros/global_position/local", 10, local_cb);                                
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//                                    ("mavros/setpoint_position/local", 10);
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//                                        ("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//                                          ("mavros/set_mode");

//     // The setpoint publishing rate MUST be faster than 2Hz.
//     ros::Rate rate(20.0);

//     // Wait for FCU connection.
//     while (ros::ok() && current_state.connected) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     geometry_msgs::PoseStamped pose;

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     ros::Time last_request(0);

//     while (ros::ok()) {
//         if (current_state.mode != "OFFBOARD" &&
//                 (ros::Time::now() - last_request > ros::Duration(5.0))) {
//             if( set_mode_client.call(offb_set_mode) &&
//                     offb_set_mode.response.mode_sent) {
//                 ROS_INFO("Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         } else {

//             if (!current_state.armed &&
//                     (ros::Time::now() - last_request > ros::Duration(5.0))) {
//                 if( arming_client.call(arm_cmd) &&
//                         arm_cmd.response.success) {
//                     ROS_INFO("Vehicle armed");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         switch (wp) {
//         case 0:    // key up
//             pose.pose.position.x = 0;
//             pose.pose.position.y = 0;
//             pose.pose.position.z = 2;
//             break;    
//         case 1:    // key up
//             pose.pose.position.x = 0;
//             pose.pose.position.y = 2;
//             pose.pose.position.z = 2;
//             break;
//         case 2:    // key down
//             pose.pose.position.x = 2;
//             pose.pose.position.y = 2;
//             pose.pose.position.z = 2;
//             break;
//         case 3:    // key right
//             pose.pose.position.x = 2;
//             pose.pose.position.y = 0;
//             pose.pose.position.z = 2;
//             break;
//         case 4:    // key left
//             pose.pose.position.x = 0;
//             pose.pose.position.y = 0;
//             pose.pose.position.z = 2;
//             break;
//         }

//         if(sqrt(pow(local_pos.pose.pose.position.x-pose.pose.position.x,2)+pow(local_pos.pose.pose.position.y-pose.pose.position.y,2)+pow(local_pos.pose.pose.position.z-pose.pose.position.z,2))<0.15)
//         {
//             wp = wp+1;

//         }   

// 		ROS_INFO("setpoint: %.1f, %.1f, %.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

//         local_pos_pub.publish(pose);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }    

// #include <iostream>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include "std_msgs/Float64.h"
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/PositionTarget.h>
// #include <unistd.h>
// #include <geometry_msgs/Pose2D.h>
// #include <mavros_msgs/CommandTOL.h>
// #include <time.h>
// #include <cmath>
// #include <math.h>
// #include <ros/duration.h>

// using namespace std;

// //Set global variables
// mavros_msgs::State current_state;
// //nav_msgs::Odometry current_pose;
// geometry_msgs::PoseStamped current_pose;
// geometry_msgs::PoseStamped pose;
// std_msgs::Float64 current_heading;
// float GYM_OFFSET;

// //get armed state
// void state_cb(const mavros_msgs::State::ConstPtr& msg)
// {
//   current_state = *msg;
//   bool connected = current_state.connected;
//   bool armed = current_state.armed;
// }
// //get current position of drone
// void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//   current_pose = *msg;
//   ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
//   // ROS_INFO("y: %f", current_pose.pose.position.y);
//   // ROS_INFO("z: %f", current_pose.pose.position.z);
// }
// // void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
// // {
// //  current_pose = *msg;
// //  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
// // }
// //get compass heading
// void heading_cb(const std_msgs::Float64::ConstPtr& msg)
// {
//   current_heading = *msg;
//   //ROS_INFO("current heading: %f", current_heading.data);
// }
// //set orientation of the drone (drone should always be level)
// void setHeading(float heading)
// {
//   heading = -heading + 90 - GYM_OFFSET;
//   float yaw = heading*(M_PI/180);
//   float pitch = 0;
//   float roll = 0;

//   float cy = cos(yaw * 0.5);
//   float sy = sin(yaw * 0.5);
//   float cr = cos(roll * 0.5);
//   float sr = sin(roll * 0.5);
//   float cp = cos(pitch * 0.5);
//   float sp = sin(pitch * 0.5);

//   float qw = cy * cr * cp + sy * sr * sp;
//   float qx = cy * sr * cp - sy * cr * sp;
//   float qy = cy * cr * sp + sy * sr * cp;
//   float qz = sy * cr * cp - cy * sr * sp;

//   pose.pose.orientation.w = qw;
//   pose.pose.orientation.x = qx;
//   pose.pose.orientation.y = qy;
//   pose.pose.orientation.z = qz;

// }
// // set position to fly to in the gym frame
// void setDestination(float x, float y, float z)
// {
//   float deg2rad = (M_PI/180);
//   float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
//   float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
//   float Z = z;
//   pose.pose.position.x = X;
//   pose.pose.position.y = Y;
//   pose.pose.position.z = Z;
//   ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "offb_node");
//   ros::NodeHandle nh;

//   // the setpoint publishing rate MUST be faster than 2Hz
//   ros::Rate rate(20.0);
//   ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
//   ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
//   ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//   ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, pose_cb);
//   ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);

//   // allow the subscribers to initialize
//   ROS_INFO("INITILIZING...");
//   for(int i=0; i<100; i++)
//   {
//     ros::spinOnce();
//     ros::Duration(0.01).sleep();
//   }
//   while(current_state.mode != "OFFBOARD")
//   {
//     ros::spinOnce();
//     ros::Duration(0.01).sleep();
//   }

//   //set the orientation of the gym
//   GYM_OFFSET = 0;
//   for (int i = 1; i <= 30; ++i) {
//     ros::spinOnce();
//     ros::Duration(0.1).sleep();
//     GYM_OFFSET += current_heading.data;
//     ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
//   }
//   GYM_OFFSET /= 30;
//   ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
//   cout << GYM_OFFSET << "\n" << endl;

//   // wait for FCU connection
//   while (ros::ok() && !current_state.connected)
//   {
//     ros::spinOnce();
//     rate.sleep();
//   }

//   // arming
//   ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//   mavros_msgs::CommandBool srv_arm_i;
//   srv_arm_i.request.value = true;
//   if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
//     ROS_INFO("ARM sent %d", srv_arm_i.response.success);
//   else
//   {
//     ROS_ERROR("Failed arming");
//     return -1;
//   }


//   //request takeoff
//   ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
//   mavros_msgs::CommandTOL srv_takeoff;
//   srv_takeoff.request.altitude = 1.5;
//   if(takeoff_cl.call(srv_takeoff)){
//     ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
//   }else{
//     ROS_ERROR("Failed Takeoff");
//     return -1;
//   }

//   sleep(10);


//   //move foreward
//   setHeading(0);
//   setDestination(0, 2, 1.5);
//   float tollorance = .35;
//   if (local_pos_pub)
//   {

//     for (int i = 10000; ros::ok() && i > 0; --i)
//     {

//       local_pos_pub.publish(pose);
//       // float percentErrorX = abs((pose.pose.position.x - current_pose.pose.position.x)/(pose.pose.position.x));
//       // float percentErrorY = abs((pose.pose.position.y - current_pose.pose.position.y)/(pose.pose.position.y));
//       // float percentErrorZ = abs((pose.pose.position.z - current_pose.pose.position.z)/(pose.pose.position.z));
//       // cout << " px " << percentErrorX << " py " << percentErrorY << " pz " << percentErrorZ << endl;
//       // if(percentErrorX < tollorance && percentErrorY < tollorance && percentErrorZ < tollorance)
//       // {
//       //   break;
//       // }
//       float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
//       float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
//       float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
//       //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
//       float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
//       cout << dMag << endl;
//       if( dMag < tollorance)
//       {
//         break;
//       }
//       ros::spinOnce();
//       ros::Duration(0.5).sleep();
//       if(i == 1)
//       {
//         ROS_INFO("Failed to reach destination. Stepping to next task.");
//       }
//     }
//     ROS_INFO("Done moving foreward.");
//   }

//   //land
//   ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
//   mavros_msgs::CommandTOL srv_land;
//   if (land_client.call(srv_land) && srv_land.response.success)
//     ROS_INFO("land sent %d", srv_land.response.success);
//   else
//   {
//     ROS_ERROR("Landing failed");
//     ros::shutdown();
//     return -1;
//   }

//   while (ros::ok())
//   {
//     ros::spinOnce();
//     rate.sleep();
//   }
//   return 0;
// }
