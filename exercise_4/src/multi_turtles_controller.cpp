//------------------------------------------------------------------------------
// INCLUDES
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "exercise_4/CustomMessage.h"


//------------------------------------------------------------------------------
// STRUCTS
struct STATE
{
  std::string robot_name;
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;
  bool sent_position;
  bool is_arrived;
  float position_x;
  float position_y;
};


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);
void customMessageCallback(const exercise_4::CustomMessage::ConstPtr& t_custom_message);


//------------------------------------------------------------------------------
// GLOBAL VARS
int g_robot_ID;
struct STATE robots_states[4];



//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_turtles_controller");
  ros::NodeHandle nh("~");
  // variables declarations

  // variables initialization

  // get parameters
  std::string robot_name;
  if (nh.getParam("robotName", robot_name))
  {
    ROS_INFO("Param found: setting name as: %s", robot_name.c_str());
  }
  else
  {
    ROS_INFO("Please pass the name: robotName:=<string>");
    return 0;
  }

  // compute integer ID from name
  g_robot_ID = (robot_name.back() - '0') - 1;
  robots_states[g_robot_ID].robot_name = robot_name.c_str();

  // subscribe and advertise to topics
  ros::Subscriber team_topic_sub = nh.subscribe("/team_topic", 20, &customMessageCallback);
  ros::Subscriber robot_pose_sub = nh.subscribe("/" + robots_states[g_robot_ID].robot_name + "/pose", 20, &poseCallback);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/" + robots_states[g_robot_ID].robot_name + "/cmd_vel", 10);
  sleep(1); // wait for adv-sub

  // send ID position and update position flag
  
  // wait for all to send their position

  // compute baricentre

  // wait until previous turtle (my_ID - 1) is arrived or previous ID is 0

  // go to baricentre
  ros::Rate loop_rate(100);
  while(ros::ok())
  {

    // create control

    // send command

    ros::spinOnce();
    loop_rate.sleep();
  }

  // update is arrived flag



  return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS DEFINITIONS
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg)
{
  robots_states[g_robot_ID].x = t_pose_msg->x;
  robots_states[g_robot_ID].y = t_pose_msg->y;
  robots_states[g_robot_ID].theta = fmod(2*M_PI + fmod(t_pose_msg->theta, 2*M_PI), 2*M_PI);
  robots_states[g_robot_ID].linear_velocity = t_pose_msg->linear_velocity;
  robots_states[g_robot_ID].angular_velocity = t_pose_msg->angular_velocity;
}

void customMessageCallback(const exercise_4::CustomMessage::ConstPtr& t_custom_message)
{

}
