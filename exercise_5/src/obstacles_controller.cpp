//------------------------------------------------------------------------------
// INCLUDES
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"


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
struct GOAL
{
  VECTOR2D vector;
  float Ka;
};


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);
VECTOR2D getAttractiveForce(GOAL t_goal, POSE t_robot_pose);
VECTOR2D getRepulsiveForce(OBSTACLE t_obstacle, POSE t_robot_pose);
float getDistance(VECTOR2D t_vector1, VECTOR2D t_vector2);
VECTOR2D getDistanceVector(VECTOR2D t_vector1, VECTOR2D t_vector2);
VECTOR2D getNormalizedVector(VECTOR2D t_vector);


//------------------------------------------------------------------------------
// GLOBAL VARS
POSE g_robot_pose;
const int g_obstacles_number = 3;
OBSTACLE g_obstacles[g_obstacles_number];
GOAL g_goal;


//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh("~");

  // variables declarations
  VECTOR2D repulsive_force_total;
  VECTOR2D attractive_force;
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
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    // sum of repulsive forces
    repulsive_force_total.x = 0;
    repulsive_force_total.y = 0;
    for (int i = 0; i < g_obstacles_number; i++)
    {
      repulsive_force_total.x += getRepulsiveForce(g_obstacles[i], g_robot_pose).x;
      repulsive_force_total.y += getRepulsiveForce(g_obstacles[i], g_robot_pose).y;
    }

    // attractive force
    attractive_force.x = 0;
    attractive_force.y = 0;
    attractive_force = getAttractiveForce(g_goal, g_robot_pose);

    // create control as sum of repulsive and attractive forces
    cmd_msg.linear.x = attractive_force.x*cos(g_robot_pose.theta)
      + repulsive_force_total.x*cos(g_robot_pose.theta)
      + attractive_force.y*sin(g_robot_pose.theta)
      + repulsive_force_total.y*sin(g_robot_pose.theta);
    cmd_msg.linear.y = attractive_force.x*sin(g_robot_pose.theta)
      + repulsive_force_total.x*sin(g_robot_pose.theta)
      + attractive_force.y*cos(g_robot_pose.theta)
      + repulsive_force_total.y*cos(g_robot_pose.theta);

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
  g_robot_pose.vector.x = t_pose_msg->x;
  g_robot_pose.vector.y = t_pose_msg->y;
  g_robot_pose.theta = fmod(2*M_PI + fmod(t_pose_msg->theta, 2*M_PI), 2*M_PI);
  g_robot_pose.linear_velocity = t_pose_msg->linear_velocity;
  g_robot_pose.angular_velocity = t_pose_msg->angular_velocity;
}

VECTOR2D getAttractiveForce(GOAL t_goal, POSE t_robot_pose)
{
  VECTOR2D attractive_force;

  float distance = getDistance(t_robot_pose.vector, t_goal.vector);
  VECTOR2D distance_vector = getDistanceVector(t_robot_pose.vector, t_goal.vector);
  VECTOR2D distance_versor = getNormalizedVector(distance_vector);

  if (distance > 1)
  {
    attractive_force.x = t_goal.Ka*distance_vector.x;
    attractive_force.y = t_goal.Ka*distance_vector.y;
  }
  else
  {
    attractive_force.x = t_goal.Ka*distance_versor.x;
    attractive_force.y = t_goal.Ka*distance_versor.y;
  }

}

VECTOR2D getRepulsiveForce(OBSTACLE t_obstacle, POSE t_robot_pose)
{
  VECTOR2D repulsive_force;
  VECTOR2D distance_vector;
  float distance;
  float force_norm;

  distance = getDistance(t_obstacle.vector, t_robot_pose.vector);
  force_norm = t_obstacle.Kr/pow(t_obstacle.gamma,2)*(1/distance - 1/t_obstacle.influence);
  distance_vector = getDistanceVector(t_obstacle.vector, t_robot_pose.vector);

  repulsive_force.x = force_norm*distance_vector.x;
  repulsive_force.y = force_norm*distance_vector.y;
}

float getDistance(VECTOR2D t_vector1, VECTOR2D t_vector2)
{
  return sqrt( pow((t_vector1.x - t_vector2.x),2) + pow((t_vector1.y - t_vector2.y),2) );
}

VECTOR2D getDistanceVector(VECTOR2D t_vector1, VECTOR2D t_vector2)
{
  VECTOR2D distance_vector;

  distance_vector.x = t_vector2.x - t_vector1.x;
  distance_vector.y = t_vector2.y - t_vector1.y;

  return distance_vector;
}

VECTOR2D getNormalizedVector(VECTOR2D t_vector)
{
  float norm = sqrt(pow(t_vector.x,2) + pow(t_vector.y,2));

  t_vector.x /= norm;
  t_vector.y /= norm;

  return t_vector;
}
