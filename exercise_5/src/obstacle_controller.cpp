//------------------------------------------------------------------------------
// INCLUDES
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "exercise_5/ObstacleMessage.h"


//------------------------------------------------------------------------------
// STRUCTS
struct VECTOR2D
{
  float x;
  float y;
};
struct POSE
{
  VECTOR2D vector;
  float theta;
  float linear_velocity;
  float angular_velocity;
};
struct OBSTACLE
{
  VECTOR2D vector;
  float Kr;
  float gamma;
  float influence;
};


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);
void obstacleMessageCallback(const exercise_5::ObstacleMessage::ConstPtr& t_obstacle_msg);
void publishObstacleMessage();


//------------------------------------------------------------------------------
// GLOBAL VARS
struct OBSTACLE g_obstacle_pose;
std::string g_obstacle_name;
int g_obstacle_ID;
ros::Publisher g_obstacle_message_pub;


//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_controller");
  ros::NodeHandle nh("~");

  // variables declarations

  // variables initialization

  // get parameters
  if (nh.getParam("robotName", g_obstacle_name))
  {
    ROS_INFO("Param found: setting name as: %s", g_obstacle_name.c_str());
  }
  else
  {
    ROS_INFO("Please pass the name in the launch file: <param name=\"robotName\" value=\"turtle2\"/>");
    return 0;
  }

  if (nh.getParam("Kr", g_obstacle_pose.Kr))
  {
    ROS_INFO("Param found: setting Kr as: %.3f", g_obstacle_pose.Kr);
  }
  else
  {
    ROS_INFO("Please pass the Kr in the launch file: <param name=\"Kr\" value=\"1\"/>");
    return 0;
  }

  if (nh.getParam("gamma", g_obstacle_pose.gamma))
  {
    ROS_INFO("Param found: setting gamma as: %.3f", g_obstacle_pose.gamma);
  }
  else
  {
    ROS_INFO("Please pass the gamma in the launch file: <param name=\"gamma\" value=\"2\"/>");
    return 0;
  }

  if (nh.getParam("influence", g_obstacle_pose.influence))
  {
    ROS_INFO("Param found: setting influence as: %.3f", g_obstacle_pose.influence);
  }
  else
  {
    ROS_INFO("Please pass the influence in the launch file: <param name=\"influence\" value=\"2\"/>");
    return 0;
  }

  // compute integer ID from name (turtle2 -> 0, turtle3 -> 1)
  g_obstacle_ID = (g_obstacle_name.back() - '0') - 2;

  // subscribe and advertise to topics
  g_obstacle_message_pub = nh.advertise<exercise_5::ObstacleMessage>("/obstacle_topic", 10);
  ros::Subscriber robot_pose_sub = nh.subscribe("/" + g_obstacle_name + "/pose", 20, &poseCallback);
  sleep(1); // wait for adv-sub

  // call pose callback
  ros::spinOnce();

  // publish obstacle message
  publishObstacleMessage();


  return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS DEFINITIONS
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg)
{
  g_obstacle_pose.vector.x = t_pose_msg->x;
  g_obstacle_pose.vector.y = t_pose_msg->y;
}


void publishObstacleMessage()
{
  exercise_5::ObstacleMessage obstacle_message;

  obstacle_message.header.stamp = ros::Time::now();
  obstacle_message.obstacle_ID = g_obstacle_ID;
  obstacle_message.x = g_obstacle_pose.vector.x;
  obstacle_message.y = g_obstacle_pose.vector.y;
  obstacle_message.Kr = g_obstacle_pose.Kr;
  obstacle_message.influence = g_obstacle_pose.influence;

  g_obstacle_message_pub.publish(obstacle_message);
}
