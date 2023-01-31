#ifndef SIM_CONTROLLER
#define SIM_CONTROLLER

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include "sim_msgs/Action.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/GetModelState.h>

enum AGENT{ROBOT, HUMAN};

void check_moving(AGENT agent, geometry_msgs::Pose& new_pose);
void wait_still_moving(AGENT agent);
void robot_action_cb(const sim_msgs::Action& msg);
void human_action_cb(const sim_msgs::Action& msg);
void manage_action(AGENT agent, const sim_msgs::Action& action);
void pick_object(AGENT agent, const std::string& obj);
void move_pose_target(AGENT agent, const geometry_msgs::Pose& pose_target);
void move_named_target(AGENT agent, const std::string& named_target);
void grab_obj(AGENT agent, const std::string& object);
void drop(AGENT agent);

geometry_msgs::Point make_point(double x, double y, double z);
geometry_msgs::Quaternion make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0);
geometry_msgs::Pose make_pose(geometry_msgs::Point p, geometry_msgs::Quaternion q);


#endif