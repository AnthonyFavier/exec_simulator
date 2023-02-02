#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/String.h>
#include <algorithm>
#include "sim_msgs/MoveArm.h"

std::string ROBOT_NAME = "";

ros::Publisher traj_pub;
moveit::planning_interface::MoveGroupInterface *move_group_interface;
std::vector<std::string> named_targets;

bool move_named_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
  if(std::find(named_targets.begin(), named_targets.end(), req.named_target) == named_targets.end())
  {
    ROS_ERROR("Named target not existing!");
    res.success = false;
  }
  else
  {
    // Set goal pose
    move_group_interface->setNamedTarget(req.named_target);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res.success = success;

    // Execute
    if(!success)
      ROS_INFO("Failed to plan...");
    else
      move_group_interface->execute(plan);
  }
  return true;
}

bool move_pose_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
  ROS_INFO("Pose target received.");

  // Set goal pose
  move_group_interface->setPoseTarget(req.pose_target);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // ROS_INFO("before planning");
  bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("after planning");
  res.success = success;

  // Execute
  if(!success)
    ROS_INFO("Failed to plan...");
  else
    move_group_interface->execute(plan);

  return true;
}

int main(int argc, char **argv)
{
  if (argc < 2)
    throw std::invalid_argument("Move_arm: Missing robot name argument!");
  else
    ROBOT_NAME = argv[1];

  ros::init(argc, argv, "move_arm_" + ROBOT_NAME);
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  move_group_interface = new moveit::planning_interface::MoveGroupInterface(ROBOT_NAME + "_arm");
  move_group_interface->setPlannerId("RRTstar");
  move_group_interface->setPlanningTime(0.3);
  move_group_interface->setGoalOrientationTolerance(3.14159265);
  move_group_interface->setMaxVelocityScalingFactor(1.0);
  move_group_interface->setMaxAccelerationScalingFactor(0.9);
  named_targets = move_group_interface->getNamedTargets();

  traj_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>(ROBOT_NAME + "_arm_controller/command", 10);
  ros::ServiceServer move_pose_target_service = node_handle.advertiseService("move_pose_target", move_pose_target_server);
  ros::ServiceServer move_named_target_service = node_handle.advertiseService("move_named_target", move_named_target_server);

  ros::Rate loop(50);
  while(ros::ok())
    loop.sleep();

  delete move_group_interface;
  ros::shutdown();
  return 0;
}
