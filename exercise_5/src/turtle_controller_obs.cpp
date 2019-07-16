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
float getNorm(VECTOR2D t_vector);
void obstacleMessageCallback(const exercise_5::ObstacleMessage::ConstPtr& t_obstacle_msg);
float getAngleError(float t_turtle_theta, float t_target_hdg);
void alignToTarget(float t_target_angle);


//------------------------------------------------------------------------------
// GLOBAL VARS
POSE g_robot_pose;
const int g_obstacles_number = 2;
OBSTACLE g_obstacles_pose[g_obstacles_number];
GOAL g_goal;
std::string g_robot_name;
ros::Publisher g_vel_pub;

//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_controller_obs");
  ros::NodeHandle nh("~");

  // variables declarations
  VECTOR2D repulsive_force_total;
  VECTOR2D attractive_force;
  geometry_msgs::Twist cmd_msg;
  float Kp_lin;

  // variables initialization
  cmd_msg.linear.x = 0;
	cmd_msg.linear.y = 0;
	cmd_msg.linear.z = 0;
	cmd_msg.angular.x = 0;
	cmd_msg.angular.y = 0;
	cmd_msg.angular.z = 0;
  Kp_lin = 0.5;

  // get parameters
  if (nh.getParam("robotName", g_robot_name))
  {
    ROS_INFO("Param found: setting name as: %s", g_robot_name.c_str());
  }
  else
  {
    ROS_INFO("Please pass the name in the launch file: <param name=\"robotName\" value=\"turtle2\"/>");
    return 0;
  }

  if (nh.getParam("goalX", g_goal.vector.x))
  {
    ROS_INFO("Param found: setting goalX as: %.2f", g_goal.vector.x);
  }
  else
  {
    ROS_INFO("Please pass the goalX in the launch file: <param name=\"goalX\" value=\"1\"/>");
    return 0;
  }

  if (nh.getParam("goalY", g_goal.vector.y))
  {
    ROS_INFO("Param found: setting goalY as: %.2f", g_goal.vector.y);
  }
  else
  {
    ROS_INFO("Please pass the goalY in the launch file: <param name=\"goalY\" value=\"1\"/>");
    return 0;
  }

  if (nh.getParam("Ka", g_goal.Ka))
  {
    ROS_INFO("Param found: setting Ka as: %.2f", g_goal.Ka);
  }
  else
  {
    ROS_INFO("Please pass the Ka in the launch file: <param name=\"Ka\" value=\"1\"/>");
    return 0;
  }

  // subscribe and advertise to topics
  ros::Subscriber obstacle_topic_sub = nh.subscribe("/obstacle_topic", 20, &obstacleMessageCallback);
  ros::Subscriber robot_pose_sub = nh.subscribe("/" + g_robot_name + "/pose", 20, &poseCallback);
  g_vel_pub = nh.advertise<geometry_msgs::Twist>("/" + g_robot_name + "/cmd_vel", 10);
  sleep(3); // wait for adv-sub
  ros::spinOnce(); // call obstacle callbacks

  // control loop
  ROS_INFO("Moving to the goal in %.2f,%.2f", g_goal.vector.x, g_goal.vector.y);
  ros::Rate loop_rate(50);
  float distance = getDistance(g_robot_pose.vector, g_goal.vector);
  float tol = 0.05;
  while(distance > tol) //ros::ok())
  {
    //ROS_INFO("obs0: x,y=%.2f,%.2f", g_obstacles_pose[0].vector.x, g_obstacles_pose[0].vector.y);
    //ROS_INFO("obs1: x,y=%.2f,%.2f", g_obstacles_pose[1].vector.x, g_obstacles_pose[1].vector.y);

    // sum of repulsive forces
    repulsive_force_total.x = 0;
    repulsive_force_total.y = 0;
    for (int i = 0; i < g_obstacles_number; i++)
    {
      VECTOR2D repulsive_force = getRepulsiveForce(g_obstacles_pose[i], g_robot_pose);
      repulsive_force_total.x += repulsive_force.x;
      repulsive_force_total.y += repulsive_force.y;
    }

    // attractive force
    attractive_force.x = 0;
    attractive_force.y = 0;
    attractive_force = getAttractiveForce(g_goal, g_robot_pose);

    // compute total force as sum of repulsive and attractive forces
    VECTOR2D total_force;
    /*
    total_force.x = attractive_force.x*cos(g_robot_pose.theta)
      + repulsive_force_total.x*cos(g_robot_pose.theta)
      + attractive_force.y*sin(g_robot_pose.theta)
      + repulsive_force_total.y*sin(g_robot_pose.theta);
    total_force.y = attractive_force.x*sin(g_robot_pose.theta)
      + repulsive_force_total.x*sin(g_robot_pose.theta)
      + attractive_force.y*cos(g_robot_pose.theta)
      + repulsive_force_total.y*cos(g_robot_pose.theta);
    */
    total_force.x = attractive_force.x + repulsive_force_total.x;
    total_force.y = attractive_force.y + repulsive_force_total.y;

    // create theta reference
    float theta_ref = atan2(total_force.y, total_force.x);
    theta_ref = fmod(2*M_PI + fmod(theta_ref, 2*M_PI), 2*M_PI);

    // align to theta reference
    alignToTarget(theta_ref);

    // move forward
    cmd_msg.linear.x = Kp_lin*getNorm(total_force);
    g_vel_pub.publish(cmd_msg);

    //ROS_INFO("control vel = %f", cmd_msg.linear.x);

    distance = getDistance(g_robot_pose.vector, g_goal.vector);


    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Goal Reached!");


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
  //ROS_INFO("attr: dist=%f", distance);
  VECTOR2D distance_vector = getDistanceVector(t_robot_pose.vector, t_goal.vector);
  //ROS_INFO("attr: dist_v=%f,%f", distance_vector.x, distance_vector.y);
  VECTOR2D distance_versor = getNormalizedVector(distance_vector);
  //ROS_INFO("attr: dist_vers=%f,%f", distance_versor.x, distance_versor.y);

  if (distance < 1)
  {
    attractive_force.x = t_goal.Ka*distance_vector.x;
    attractive_force.y = t_goal.Ka*distance_vector.y;
  }
  else
  {
    attractive_force.x = t_goal.Ka*distance_versor.x;
    attractive_force.y = t_goal.Ka*distance_versor.y;
  }

  //ROS_INFO("attr: attr_force_v=%f,%f", attractive_force.x, attractive_force.y);

  return attractive_force;
}

VECTOR2D getRepulsiveForce(OBSTACLE t_obstacle, POSE t_robot_pose)
{
  VECTOR2D repulsive_force;
  VECTOR2D distance_vector;
  float distance;
  float force_norm;

  distance = getDistance(t_obstacle.vector, t_robot_pose.vector);
  //ROS_INFO("rep: dist=%f", distance);
  force_norm = t_obstacle.Kr/(pow(distance,2))
    //*(pow((1/distance - 1/t_obstacle.influence), (t_obstacle.gamma - 1)));
    *(1/distance - 1/t_obstacle.influence);
  //ROS_INFO("rep: f_norm=%f", force_norm);
  //ROS_INFO("rep: f_norm=%.3f", force_norm);
  //ROS_INFO("rep: f_norm=%f", force_norm);
  distance_vector = getDistanceVector(t_obstacle.vector, t_robot_pose.vector);
  //ROS_INFO("rep: dist_v=%f,%f", distance_vector.x, distance_vector.y);

  if (distance < t_obstacle.influence)
  {
    repulsive_force.x = force_norm*distance_vector.x;
    repulsive_force.y = force_norm*distance_vector.y;
  }
  else
  {
    repulsive_force.x = 0.0;
    repulsive_force.y = 0.0;
  }

  //ROS_INFO("rep: rep_force_v f=%f,%f", repulsive_force.x, repulsive_force.y);

  return repulsive_force;
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
  float norm = getNorm(t_vector);

  t_vector.x /= norm;
  t_vector.y /= norm;

  return t_vector;
}

float getNorm(VECTOR2D t_vector)
{
  return sqrt(pow(t_vector.x,2) + pow(t_vector.y,2));
}

void obstacleMessageCallback(const exercise_5::ObstacleMessage::ConstPtr& t_obstacle_msg)
{
  int obs_ID = t_obstacle_msg->obstacle_ID;

  g_obstacles_pose[obs_ID].vector.x = t_obstacle_msg->x;
  g_obstacles_pose[obs_ID].vector.y = t_obstacle_msg->y;
  g_obstacles_pose[obs_ID].Kr = t_obstacle_msg->Kr;
  g_obstacles_pose[obs_ID].gamma = t_obstacle_msg->gamma;
  g_obstacles_pose[obs_ID].influence = t_obstacle_msg->influence;
}

float getAngleError(float t_turtle_theta, float t_target_hdg)
{
  float err = t_target_hdg-t_turtle_theta;
  if (err < M_PI && err > -M_PI)
	{
		err = t_target_hdg - t_turtle_theta;
	}
	else if (err > M_PI)
	{
		err -= 2*M_PI;
	}
	else if (err <= -M_PI)
	{
		err += 2*M_PI;
	}

  return err;
}

void alignToTarget(float t_target_angle)
{
  //ROS_INFO("aligning to angle=%.2f", t_target_angle);

  float Kp_ang = 1.2, tol = 0.05;
  float angle_error = getAngleError(g_robot_pose.theta, t_target_angle);
  //ROS_INFO("angle_err=%.3f", angle_error);

  geometry_msgs::Twist cmd_msg;
  cmd_msg.linear.x = 0;
	cmd_msg.linear.y = 0;
	cmd_msg.linear.z = 0;
	cmd_msg.angular.x = 0;
	cmd_msg.angular.y = 0;
	cmd_msg.angular.z = 0;

  ros::Rate loop_rate(100);
  while (fabs(angle_error) > tol)
  {
		angle_error = getAngleError(g_robot_pose.theta, t_target_angle);

		// apply velocity
		cmd_msg.angular.z = Kp_ang*angle_error;

		// publish command
		g_vel_pub.publish(cmd_msg);

    //ROS_INFO("aligning: angle_err=%.3f", angle_error);


	  ros::spinOnce();
		loop_rate.sleep();
	}

	// stop turtle
	cmd_msg.angular.z = 0;
  g_vel_pub.publish(cmd_msg);

  //ROS_INFO("aligned!");
}
