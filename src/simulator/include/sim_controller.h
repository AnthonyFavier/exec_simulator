#ifndef SIM_CONTROLLER
#define SIM_CONTROLLER

#include <ros/ros.h>
#include <thread>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include "sim_msgs/Action.h"
#include "sim_msgs/MoveArm.h"
#include "sim_msgs/AttachObj.h"
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

#define ROBOT_ATTACH_MODEL_NAME "panda1"
#define ROBOT_ATTACH_LINK_NAME "panda1_link7"
#define HUMAN_ATTACH_MODEL_NAME "human_hand"
#define HUMAN_ATTACH_LINK_NAME "human_hand_link"


enum AGENT{ROBOT, HUMAN};

/* UTILS */
geometry_msgs::Point make_point(double x, double y, double z);
geometry_msgs::Quaternion make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0);
geometry_msgs::Quaternion make_quaternion_RPY(double r = 0.0, double p = 0.0, double y = 0.0);
geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q);
void show_pose(geometry_msgs::Pose pose);
std::string get_agent_str(AGENT agent);

/* HIGH LEVEL ACTIONS */
void Pick(AGENT agent, const std::string &color, const std::string &side);
void PlacePose(AGENT agent, geometry_msgs::Pose pose);
void PlaceLocation(AGENT agent, const std::string &location);
void BePassive(AGENT agent);
void Pushing(AGENT agent);
void OpenBox(AGENT agent);
void DropCube(AGENT agent);

/* LOW LEVEL ACTIONS */
void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target, bool human_home=false);
void move_location_target(AGENT agent, const std::string &loc_name);
void move_obj_target(AGENT agent, const std::string &obj_name);
void move_home(AGENT agent);
void move_named_target(AGENT agent, const std::string &named_target);
void grab_obj(AGENT agent, const std::string &object);
void drop(AGENT agent, const std::string &object);
void set_obj_rpy(AGENT agent, std::string obj_name, float r, float p, float y);
void set_obj_pose(AGENT agent, std::string obj_name, geometry_msgs::Pose pose);
void delta_move_obj(AGENT agent, std::string obj_name, geometry_msgs::Pose delta_move);

/* CALLBACKS */
void robot_action_cb(const sim_msgs::Action &msg);
void human_action_cb(const sim_msgs::Action &msg);
void manage_action(AGENT agent, const sim_msgs::Action &action);
void r_home_cb(std_msgs::Empty msg);

std::string compute_event_name(AGENT agent, sim_msgs::Action action, bool start);
void send_visual_signal_action_start(AGENT agent, sim_msgs::Action action);
void send_visual_signal_action_over(AGENT agent, sim_msgs::Action action);


void home_agents();

#endif