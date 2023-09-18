#ifndef SIM_CONTROLLER
#define SIM_CONTROLLER

#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include "sim_msgs/Action.h"
#include "sim_msgs/MoveArm.h"
#include "sim_msgs/EventLog.h"
#include "sim_msgs/Signal.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_msgs/SetLinkState.h"
#include "sim_msgs/HeadCmd.h"

#define ROBOT_ATTACH_MODEL_NAME "tiago"
#define ROBOT_ATTACH_LINK_NAME "arm_7_link"
#define HUMAN_ATTACH_MODEL_NAME "human_hand"
#define HUMAN_ATTACH_LINK_NAME "human_hand_link"


enum AGENT{ROBOT, HUMAN};

// ************************* LOW LEVEL ACTIONS **************************** //
void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target, bool human_home = false);
void move_location_target(AGENT agent, const std::string &loc_name);
void move_obj_target(AGENT agent, const std::string &obj_name);
void move_home(AGENT agent);
void move_named_target(AGENT agent, const std::string &named_target);
void grab_obj(AGENT agent, const std::string &object);
void drop(AGENT agent, const std::string &object);
void set_obj_rpy(AGENT agent, std::string obj_name, float r, float p, float y);
void set_obj_pose(AGENT agent, std::string obj_name, geometry_msgs::Pose pose);
void adjust_obj_pose(AGENT agent, std::string obj_name, geometry_msgs::Pose pose);
void delta_move_obj(AGENT agent, std::string obj_name, geometry_msgs::Pose delta_move);
void robot_head_follow_pose(AGENT agent, geometry_msgs::Point pose);
void robot_head_follow_obj(AGENT agent, std::string obj_name);
void robot_head_follow_human(AGENT agent);
void robot_head_follow_stack(AGENT agent);


// ****************************** UTILS ******************************* //
geometry_msgs::Point make_point(double x, double y, double z);
geometry_msgs::Quaternion make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0);
geometry_msgs::Quaternion make_quaternion_RPY(double r = 0.0, double p = 0.0, double y = 0.0);
geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q);
void show_pose(geometry_msgs::Pose pose);
std::string get_agent_str(AGENT agent);
bool isRobot(AGENT agent);


// ****************************** CALLBACKS ******************************* //
void robot_action_cb(const sim_msgs::Action &msg);
void human_action_cb(const sim_msgs::Action &msg);
void r_home_cb(std_msgs::Empty msg);
void h_home_cb(std_msgs::Empty msg);
void h_start_moving_cb(std_msgs::Empty msg);
void r_start_moving_cb(std_msgs::Empty msg);

// ************************************************************************ //
void send_visual_signal_action_over(AGENT agent, sim_msgs::Action action);
bool reset_world_server(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool go_idle_pose_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
void go_idle_pose_cb(const std_msgs::Empty &msg);
bool go_home_pose_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
void go_home_pose_cb(const std_msgs::Empty &msg);
void home_agents();



#endif