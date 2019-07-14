//------------------------------------------------------------------------------
// INCLUDES
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "exercise_2/CustomControl.h"


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


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void customControlCallback(const exercise_2::CustomControl::ConstPtr& t_custom_control_msg);
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);
float applySaturation(float t_input, float t_sat_max, float t_sat_min);


//------------------------------------------------------------------------------
// GLOBAL VARS
exercise_2::CustomControl g_custom_control_msg;
struct POSE g_robot_pose;
bool g_received_command;

//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh("~");

  // variables declarations
  float lin_vel_sat_min, lin_vel_sat_max, ang_vel_sat_min, ang_vel_sat_max;
  geometry_msgs::Twist cmd_msg;

  // variables initialization
  cmd_msg.linear.x = 0;
	cmd_msg.linear.y = 0;
	cmd_msg.linear.z = 0;
	cmd_msg.angular.x = 0;
	cmd_msg.angular.y = 0;
	cmd_msg.angular.z = 0;
  g_received_command = false;

  // get parameters
  if (nh.getParam("lin_vel_sat_min", lin_vel_sat_min))
  {
    ROS_INFO("Param found: setting lin_vel_sat_min as: %f", lin_vel_sat_min);
  }
  else
  {
    lin_vel_sat_min = -2;
    ROS_INFO("Param not found: setting lin_vel_sat_min as: %f", lin_vel_sat_min);
  }

  if (nh.getParam("lin_vel_sat_max", lin_vel_sat_max))
  {
    ROS_INFO("Param found: setting lin_vel_sat_max as: %f", lin_vel_sat_max);
  }
  else
  {
    lin_vel_sat_max = 2;
    ROS_INFO("Param not found: setting lin_vel_sat_max as: %f", lin_vel_sat_max);
  }

  if (nh.getParam("ang_vel_sat_min", ang_vel_sat_min))
  {
    ROS_INFO("Param found: setting ang_vel_sat_min as: %f", ang_vel_sat_min);
  }
  else
  {
    ang_vel_sat_min = -2;
    ROS_INFO("Param not found: setting ang_vel_sat_min as: %f", ang_vel_sat_min);
  }

  if (nh.getParam("ang_vel_sat_max", ang_vel_sat_max))
  {
    ROS_INFO("Param found: setting ang_vel_sat_max as: %f", ang_vel_sat_max);
  }
  else
  {
    ang_vel_sat_max = 2;
    ROS_INFO("Param not found: setting ang_vel_sat_max as: %f", ang_vel_sat_max);
  }

  // subscribe and advertise to topics
  ros::Subscriber custom_control_sub = nh.subscribe("/custom_control", 20, &customControlCallback);
  ros::Subscriber robot_pose_sub = nh.subscribe("/turtle1/pose", 20, &poseCallback);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  sleep(1); // wait for adv-sub

  ROS_INFO("Ready to receive command");
  ROS_INFO("Type the custom control as explained in the README\n");


  // control turtle loop
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    if (g_received_command)
    {
      // apply saturation
      cmd_msg.linear.x = applySaturation(g_custom_control_msg.lin_vel_ref, lin_vel_sat_max, lin_vel_sat_min);
      cmd_msg.angular.z = applySaturation(g_custom_control_msg.ang_vel_ref, ang_vel_sat_max, ang_vel_sat_min);

      // send command
      vel_pub.publish(cmd_msg);
      g_received_command = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS DEFINITIONS
void customControlCallback(const exercise_2::CustomControl::ConstPtr& t_custom_control_msg)
{
  g_custom_control_msg.lin_vel_ref = t_custom_control_msg->lin_vel_ref;
  g_custom_control_msg.ang_vel_ref = t_custom_control_msg->ang_vel_ref;

  g_received_command = true;
}

void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg)
{
  g_robot_pose.x = t_pose_msg->x;
  g_robot_pose.y = t_pose_msg->y;
  g_robot_pose.theta = t_pose_msg->theta;
  g_robot_pose.linear_velocity = t_pose_msg->linear_velocity;
  g_robot_pose.angular_velocity = t_pose_msg->angular_velocity;
}

float applySaturation(float t_input, float t_sat_max, float t_sat_min)
{
  if (t_input > t_sat_max)
  {
    return t_sat_max;
  }
  else if (t_input < t_sat_min)
  {
    return t_sat_min;
  }

  return t_input;
}
