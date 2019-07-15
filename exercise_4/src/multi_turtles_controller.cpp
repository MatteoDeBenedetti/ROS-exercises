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
  int robot_ID;
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;
  bool sent_position;
  bool is_arrived;
};


//------------------------------------------------------------------------------
// FUNCTIONS PROTOTYPES
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg);
void customMessageCallback(const exercise_4::CustomMessage::ConstPtr& t_custom_message);
void publishCustomMessage(bool t_sent_position, bool t_is_arrived, float t_x, float t_y);
void waitForPositionsSent();
void computeBarycenter(float* barycenter);
void waitForPreviousArrived();
void moveToTarget(float t_target_x, float t_target_y);
float getDistance(STATE t_robot_state, float t_target_x, float t_target_y);
float getOrientationError(STATE t_robot_state, float t_target_x, float t_target_y);


//------------------------------------------------------------------------------
// GLOBAL VARS
int g_robot_ID;
std::string g_robot_name;
struct STATE g_robots_states[4];
bool g_all_positions_sent = false;
bool g_all_arrived = false;
int g_robots_number = 4;
ros::Publisher g_vel_pub;
ros::Publisher g_team_topic_pub;


//------------------------------------------------------------------------------
// MAIN FUNCTION
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_turtles_controller");
  ros::NodeHandle nh("~");

  // variables declarations
  float barycenter[2];

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
  g_robot_name = robot_name.c_str();

  // subscribe and advertise to topics
  ros::Subscriber team_topic_sub = nh.subscribe("/team_topic", 20, &customMessageCallback);
  g_team_topic_pub = nh.advertise<exercise_4::CustomMessage>("/team_topic", 10);
  ros::Subscriber robot_pose_sub = nh.subscribe("/" + g_robot_name + "/pose", 20, &poseCallback);
  g_vel_pub = nh.advertise<geometry_msgs::Twist>("/" + g_robot_name + "/cmd_vel", 10);
  sleep(2); // wait for adv-sub
	ros::spinOnce(); // Call the Callbacks to get current position
	sleep(1);

  // send ID position and update position flag
  publishCustomMessage(false, false, 0.0, 0.0);
  publishCustomMessage(true, false, g_robots_states[g_robot_ID].x, g_robots_states[g_robot_ID].y);

  // wait for all to send their position
  waitForPositionsSent();

  // compute barycenter
  computeBarycenter(barycenter);

  // wait until previous turtle (my_ID - 1) is arrived or previous ID is 0
  waitForPreviousArrived();

  // go to barycenter
  moveToTarget(barycenter[0], barycenter[1]);

  // update is arrived flag
  publishCustomMessage(true, true, g_robots_states[g_robot_ID].x, g_robots_states[g_robot_ID].y);

  return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS DEFINITIONS
void poseCallback(const turtlesim::Pose::ConstPtr& t_pose_msg)
{
  g_robots_states[g_robot_ID].x = t_pose_msg->x;
  g_robots_states[g_robot_ID].y = t_pose_msg->y;
  g_robots_states[g_robot_ID].theta = fmod(2*M_PI + fmod(t_pose_msg->theta, 2*M_PI), 2*M_PI);
  g_robots_states[g_robot_ID].linear_velocity = t_pose_msg->linear_velocity;
  g_robots_states[g_robot_ID].angular_velocity = t_pose_msg->angular_velocity;
}

void customMessageCallback(const exercise_4::CustomMessage::ConstPtr& t_custom_message)
{
  int msg_ID = t_custom_message->robot_ID;

  g_robots_states[msg_ID].sent_position = t_custom_message->sent_position;
  g_robots_states[msg_ID].is_arrived = t_custom_message->is_arrived;
  g_robots_states[msg_ID].x = t_custom_message->x;
  g_robots_states[msg_ID].y = t_custom_message->y;

  // check if all robots sent position
  if (!g_all_positions_sent)
  {
    int ready_count = 0;
    for (int i = 0; i < g_robots_number; i++)
    {
      if(g_robots_states[i].sent_position)
      {
        ready_count++;
      }
    }

    if(ready_count == g_robots_number)
    {
      ROS_INFO("%d: All positions sent", g_robot_ID);
      g_all_positions_sent = true;
    }
  }

  // check if all robots have arrived
  if (!g_all_arrived)
  {
    int ready_count = 0;
    for (int i = 0; i < g_robots_number; i++)
    {
      if(g_robots_states[i].is_arrived)
      {
        ready_count++;
      }
    }

    if(ready_count == g_robots_number)
    {
      ROS_INFO("%d: All robots arrived!", g_robot_ID);
      g_all_arrived = true;
    }
  }

}

void publishCustomMessage(bool t_sent_position, bool t_is_arrived, float t_x, float t_y)
{
  exercise_4::CustomMessage custom_message;

  custom_message.header.stamp = ros::Time::now();
  custom_message.robot_ID = g_robot_ID;
  custom_message.sent_position = t_sent_position;
  custom_message.is_arrived = t_is_arrived;
  custom_message.x = t_x;
  custom_message.y = t_y;

  g_team_topic_pub.publish(custom_message);
}

void waitForPositionsSent()
{
  ros::Rate loopRate(1);
  while (!g_all_positions_sent)
  {
    ROS_INFO("%d: waiting for others to send pos...", g_robot_ID);
    //publishFlags(true, false);

    ros::spinOnce();
    loopRate.sleep();
  }
}

void computeBarycenter(float* barycenter)
{
  float bar_x = 0, bar_y = 0;
  for (int i = 0; i < g_robots_number; i++)
  {
    bar_x += g_robots_states[i].x;
    bar_y += g_robots_states[i].y;
  }
  barycenter[0] = bar_x/g_robots_number;
  barycenter[1] = bar_y/g_robots_number;

  ROS_INFO("%d: barycenter computed = %.2f,%.2f", g_robot_ID, barycenter[0], barycenter[1]);
}

void waitForPreviousArrived()
{
  bool is_previous_arrived = false;

  if (g_robot_ID == 0) //it is the first robot
  {
    //is_previous_arrived = true;
    return;
  }

  ros::Rate loopRate(1);
  while (!is_previous_arrived)
  {
    if (g_robots_states[g_robot_ID-1].is_arrived)
    {
      //is_previous_arrived = true;
      return;
    }

    ROS_INFO("%d: waiting for %d to arrive...", g_robot_ID, g_robot_ID-1);

    ros::spinOnce();
    loopRate.sleep();
  }
}

void moveToTarget(float t_target_x, float t_target_y)
{
  ROS_INFO("%d: moving to barycenter = %.2f,%.2f", g_robot_ID, t_target_x, t_target_y);

  float Kp_lin = .3, Kp_ang = .5, tol = 0.01;
  float distance, orientation;

  geometry_msgs::Twist cmd_msg;
  cmd_msg.linear.x = 0;
	cmd_msg.linear.y = 0;
	cmd_msg.linear.z = 0;
	cmd_msg.angular.x = 0;
	cmd_msg.angular.y = 0;
	cmd_msg.angular.z = 0;

  ros::Rate loopRate(10);
	do
	{
		orientation = getOrientationError(g_robots_states[g_robot_ID], t_target_x, t_target_y);

		// apply velocity
		cmd_msg.linear.x = 0.1;
		cmd_msg.angular.z = Kp_ang*orientation;

		// publish command
		g_vel_pub.publish(cmd_msg);

	  ros::spinOnce();
		loopRate.sleep();
	} while(abs(orientation) > tol);

  do
	{
		distance = getDistance(g_robots_states[g_robot_ID], t_target_x, t_target_y);
    orientation = getOrientationError(g_robots_states[g_robot_ID], t_target_x, t_target_y);

		// apply velocity
		cmd_msg.linear.x = Kp_lin*distance;
		cmd_msg.angular.z = Kp_ang*orientation;

		// publish command
		g_vel_pub.publish(cmd_msg);

		ros::spinOnce();
		loopRate.sleep();
	} while(distance > tol);

	// stop turtle
	cmd_msg.linear.x = 0;
	cmd_msg.angular.z = 0;
  g_vel_pub.publish(cmd_msg);

	ROS_INFO("%d: Goal reached!", g_robot_ID);
}

float getDistance(STATE t_robot_state, float t_target_x, float t_target_y)
{
  return sqrt( pow((t_robot_state.x - t_target_x),2) + pow((t_robot_state.y - t_target_y),2) );
}

float getOrientationError(STATE t_robot_state, float t_target_x, float t_target_y)
{
  float turtle_theta = t_robot_state.theta;

  float target_hdg = atan2(t_target_y - t_robot_state.y,  t_target_x - t_robot_state.x);
  target_hdg = fmod(2*M_PI + fmod(target_hdg, 2*M_PI), 2*M_PI);

  float err = target_hdg-turtle_theta;
  if (err < M_PI && err > -M_PI)
	{
		err = target_hdg - turtle_theta;
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
