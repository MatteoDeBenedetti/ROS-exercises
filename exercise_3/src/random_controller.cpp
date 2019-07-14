//------------------------------------------------------------------------------
// INCLUDES
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>
#include <time.h>

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
  float Kp_theta;

  // variables initialization
  cmd_msg.linear.x = 0;
	cmd_msg.linear.y = 0;
	cmd_msg.linear.z = 0;
	cmd_msg.angular.x = 0;
	cmd_msg.angular.y = 0;
	cmd_msg.angular.z = 0;
  Kp_theta = 1;

  // get parameters

  // subscribe and advertise to topics
  ros::Subscriber robot_pose_sub = nh.subscribe("/turtle1/pose", 20, &poseCallback);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  sleep(1); // wait for adv-sub

  // control loop
  srand(g_seed);
  // srand(time(NULL));
  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    // create random control
    cmd_msg.linear.x = createRandomNumber(g_seed, -2, 2);
    cmd_msg.angular.z = createRandomNumber(g_seed, -2, 2);

    // limit to upper half
    if (g_robot_pose.y <= 6.0) /* && (
      (g_robot_pose.theta < M_PI && g_robot_pose.linear_velocity < 0) ||
      (g_robot_pose.theta < 2*M_PI && g_robot_pose.theta > M_PI && g_robot_pose.linear_velocity > 0) ))
      */
    {
      ROS_INFO("inside if near obstacle");
      // reorient turtle upwards
      cmd_msg.linear.x = 0;
      ros::Rate loop_rate2(100);
      while (fabs(g_robot_pose.theta - M_PI/2.0) > 0.1)
      {
        ROS_INFO("correcting theta. err:%.2f", fabs(g_robot_pose.theta - M_PI/2.0));
      	cmd_msg.angular.z = Kp_theta*(M_PI/2 - g_robot_pose.theta);
        vel_pub.publish(cmd_msg);

        ros::spinOnce();
        loop_rate2.sleep();
      }
      ROS_INFO("theta aligned, now go up");
      cmd_msg.angular.z = 0;
      cmd_msg.linear.x = 1;
      vel_pub.publish(cmd_msg);
    }
    ROS_INFO("out of obstacle near if");

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
  g_robot_pose.theta = fmod(2*M_PI + fmod(t_pose_msg->theta, 2*M_PI), 2*M_PI);
  g_robot_pose.linear_velocity = t_pose_msg->linear_velocity;
  g_robot_pose.angular_velocity = t_pose_msg->angular_velocity;

  //ROS_INFO("theta clamped: %f", g_robot_pose.theta);
}

float createRandomNumber(int t_seed, int min, int max)
{
  float rand_number;
  //rand_number = rand() % ((max+abs(min)+1)*10 + min*10)/10;
  rand_number = (rand() % (max*10 + 1))/10.0*2.0 + min;
  ROS_INFO("rand number generated: %f\n", rand_number);
  return rand_number;
}
