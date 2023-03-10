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


enum AGENT{ROBOT, HUMAN};

void check_moving(AGENT agent, geometry_msgs::Pose& new_pose);
void wait_still_moving(AGENT agent);
void robot_action_cb(const sim_msgs::Action& msg);
void human_action_cb(const sim_msgs::Action& msg);
void manage_action(AGENT agent, const sim_msgs::Action& action);

void pick(AGENT agent, const std::string &obj);
void place_pose(AGENT agent, const geometry_msgs::Pose &pose);
void place_location(AGENT agent, const std::string &location);

void move_pose_target(AGENT agent, const geometry_msgs::Pose& pose_target);
void move_location_target(AGENT agent, const std::string &loc_name);
void move_obj_target(AGENT agent, const std::string &obj_name);
void move_named_target(AGENT agent, const std::string& named_target);
void grab_obj(AGENT agent, const std::string& object);
void drop(AGENT agent);
void delta_move_obj(AGENT agent, std::string obj_name, geometry_msgs::Pose delta_move);
void set_obj_pose(AGENT agent, std::string obj_name, geometry_msgs::Pose pose);
void set_obj_rpy(AGENT agent, std::string obj_name, float r, float p, float y);


geometry_msgs::Point make_point(double x, double y, double z);
geometry_msgs::Quaternion make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0);
geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q);


#endif