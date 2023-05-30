#ifndef SIM_CONTROLLER
#define SIM_CONTROLLER

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include "sim_msgs/Action.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/GetModelState.h>
#include "sim_msgs/MoveArm.h"
#include "sim_msgs/AttachObj.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>
#include "gazebo_ros_link_attacher/Attach.h"

enum AGENT{ROBOT, HUMAN};

/* UTILS */
geometry_msgs::Point make_point(double x, double y, double z);
geometry_msgs::Quaternion make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0);
geometry_msgs::Quaternion make_quaternion_RPY(double r = 0.0, double p = 0.0, double y = 0.0);
geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q);
void show_pose(geometry_msgs::Pose pose);
std::string get_agent_str(AGENT agent);

/* HIGH LEVEL ACTIONS */
void Pick(AGENT agent, const std::string &obj);
void PlacePose(AGENT agent, geometry_msgs::Pose pose, const std::string &obj);
void PlaceLocation(AGENT agent, const std::string &location, const std::string &obj);
void Wait(AGENT agent);
void Pushing(AGENT agent);
void OpenBox(AGENT agent);
void DropCube(AGENT agent, const std::string &obj);

/* LOW LEVEL ACTIONS */
void move_pose_target(AGENT agent, const geometry_msgs::Pose &pose_target);
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
bool reset_obj_server(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

#endif