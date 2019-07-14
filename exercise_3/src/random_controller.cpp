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


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);
float createRandomNumber(int t_seed, int min, int max);


//------------------------------------------------------------------------------
// GLOBAL VARS
struct POSE g_robot_pose;
int g_seed = 20071969;


//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_controller");
  ros::NodeHandle nh("~");

  // variables declarations
  geometry_msgs::Twist cmd_msg;

  // variables initialization
  cmd_msg.linear.x = 0;
	cmd_msg.linear.y = 0;
	cmd_msg.linear.z = 0;
	cmd_msg.angular.x = 0;
	cmd_msg.angular.y = 0;
	cmd_msg.angular.z = 0;

  // get parameters

  // subscribe and advertise to topics
  ros::Subscriber robot_pose_sub = nh.subscribe("/turtle1/pose", 20, &poseCallback);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  sleep(1); // wait for adv-sub

  // control loop
  srand(g_seed);
  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    // create random control
    cmd_msg.linear.x = createRandomNumber(g_seed, -5, 5);
    cmd_msg.angular.z = createRandomNumber(g_seed, -5, 5);

    // send command
    vel_pub.publish(cmd_msg);

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
  g_robot_pose.theta = t_pose_msg->theta;
  g_robot_pose.linear_velocity = t_pose_msg->linear_velocity;
  g_robot_pose.angular_velocity = t_pose_msg->angular_velocity;
}

float createRandomNumber(int t_seed, int min, int max)
{
  float rand_number;
  rand_number = rand() % ((max+abs(min)+1)*10 + min*10)/10;
  ROS_INFO("rand number generated: %f\n", rand_number);
  return rand_number;
}
