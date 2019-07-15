//------------------------------------------------------------------------------
// INCLUDES
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"


//------------------------------------------------------------------------------
// STRUCTS
struct POSE
{
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;
};
struct OBSTACLE
{
  float x;
  float y;
  float Kr;
  float gamma;
  float influence;
};
struct GOAL
{
  float x;
  float y;
  float Ka;
};

//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);


//------------------------------------------------------------------------------
// GLOBAL VARS
struct POSE g_robot_pose;


//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh("~");

  // variables declarations

  // variables initialization

  // get parameters

  // subscribe and advertise to topics
  ros::Subscriber custom_control_sub = nh.subscribe("/custom_control", 20, &customControlCallback);
  ros::Subscriber robot_pose_sub = nh.subscribe("/turtle1/pose", 20, &poseCallback);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  sleep(1); // wait for adv-sub

  // control loop
  ros::Rate loop_rate(100);
  while(ros::ok())
  {

    // create control

    // send command


    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS DEFINITIONS
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg)
{
  g_robot_pose.x = t_pose_msg->x;
  g_robot_pose.y = t_pose_msg->y;
  g_robot_pose.theta = fmod(2*M_PI + fmod(t_pose_msg->theta, 2*M_PI), 2*M_PI);
  g_robot_pose.linear_velocity = t_pose_msg->linear_velocity;
  g_robot_pose.angular_velocity = t_pose_msg->angular_velocity;
}
