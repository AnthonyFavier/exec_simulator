#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/String.h>
#include <algorithm>

std::string ROBOT_NAME = "";

ros::Publisher traj_pub;
moveit::planning_interface::MoveGroupInterface *move_group_interface;
std::vector<std::string> named_targets;

void move_goal_pose_cb(const geometry_msgs::Pose &goal_pose)
{
  ROS_INFO("Pose target received.");

  // Set goal pose
  move_group_interface->setPoseTarget(goal_pose);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ROS_INFO("before planning");
  bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("after planning");

  // Execute
  if(!success)
    ROS_INFO("Failed to plan...");
  else
  {
    ROS_INFO("Success to plan");
    traj_pub.publish(plan.trajectory_.joint_trajectory);
  }
}

void move_named_target(const std_msgs::String &named_target)
{
  ROS_INFO("Named target received.");

  if(std::find(named_targets.begin(), named_targets.end(), named_target.data) == named_targets.end())
    ROS_ERROR("Named target not existing!");
  else
  {
    // Set Goal Pose
    move_group_interface->setNamedTarget(named_target.data);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    if(!success)
      ROS_INFO("Failed to plan...");
    else
    {
      ROS_INFO("Success to plan");
      traj_pub.publish(plan.trajectory_.joint_trajectory);
    }
  }
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
  move_group_interface->setGoalOrientationTolerance(3.14159265);
  move_group_interface->setMaxVelocityScalingFactor(1.0);
  move_group_interface->setMaxAccelerationScalingFactor(0.9);
  named_targets = move_group_interface->getNamedTargets();

  traj_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/" + ROBOT_NAME + "_arm_controller/command", 1);
  ros::Subscriber goal_pose_sub = node_handle.subscribe("/" + ROBOT_NAME + "_move_goal_pose", 1, move_goal_pose_cb);
  ros::Subscriber named_target_sub = node_handle.subscribe("/" + ROBOT_NAME + "_move_named_target", 1, move_named_target);

  // ros::spin();
  ros::Rate loop(50);
  while(ros::ok())
    loop.sleep();

  delete move_group_interface;
  ros::shutdown();
  return 0;
}
