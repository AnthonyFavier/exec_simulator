#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/String.h>
#include <algorithm>
#include "sim_msgs/MoveArm.h"
#include "moveit_msgs/Constraints.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetLinkState.h"
#include "moveit_msgs/RobotState.h"
#include "moveit/robot_state/robot_state.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>


std::string ROBOT_NAME = "";

ros::Publisher traj_pub;
moveit::planning_interface::MoveGroupInterface *move_group_interface;
std::vector<std::string> named_targets;
ros::ServiceClient set_link_state_client;
ros::ServiceClient get_link_state_client;

planning_scene::PlanningScene *p_scene;


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
    res.success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    if(!res.success)
      ROS_INFO("Failed to plan...");
    else
    {
      // move_group_interface->execute(plan);

      std::cout << "ofineo" << std::endl;

    }
  }
  return true;
}

bool move_pose_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
  ROS_INFO("Pose target received.");

  gazebo_msgs::GetLinkState get_link_state;
  get_link_state.request.link_name = "human_hand_link";
  get_link_state_client.call(get_link_state);

  std::vector<double> values;
  values.push_back(get_link_state.response.link_state.pose.position.x);
  values.push_back(get_link_state.response.link_state.pose.position.y);
  values.push_back(get_link_state.response.link_state.pose.position.z);
  values.push_back(get_link_state.response.link_state.pose.orientation.x);
  values.push_back(get_link_state.response.link_state.pose.orientation.y);
  values.push_back(get_link_state.response.link_state.pose.orientation.z);
  values.push_back(get_link_state.response.link_state.pose.orientation.w);
  auto startState = (*p_scene).getCurrentState();
  startState.setJointGroupPositions("hand", values);
  move_group_interface->setStartState(startState);

  // Set goal pose
  // move_group_interface->setPoseTarget(req.pose_target);
  std::vector<double> joint_values;
  joint_values.push_back(req.pose_target.position.x);
  joint_values.push_back(req.pose_target.position.y);
  joint_values.push_back(req.pose_target.position.z);
  joint_values.push_back(req.pose_target.orientation.x);
  joint_values.push_back(req.pose_target.orientation.y);
  joint_values.push_back(req.pose_target.orientation.z);
  joint_values.push_back(req.pose_target.orientation.w);
  // joint_values.push_back(3.0);
  // joint_values.push_back(0.0);
  // joint_values.push_back(0.0);
  // joint_values.push_back(0.0);
  // joint_values.push_back(0.0);
  // joint_values.push_back(0.0);
  // joint_values.push_back(1.0);
  move_group_interface->setJointValueTarget(joint_values);


  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  res.success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Execute
  if(!res.success)
    ROS_INFO("Failed to plan...");
  else
  {
    // move_group_interface->execute(plan);

    std::cout << "ofineo" << std::endl;
    // ros::Time start_exec_time = ros::

    ros::Rate rate(10);
    gazebo_msgs::SetLinkState link_state;
    link_state.request.link_state.link_name = "human_hand_link";
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::iterator it;
    for(it=plan.trajectory_.multi_dof_joint_trajectory.points.begin(); it!=plan.trajectory_.multi_dof_joint_trajectory.points.end(); it++)
    {
      link_state.request.link_state.pose.position.x = (*it).transforms[0].translation.x;
      link_state.request.link_state.pose.position.y = (*it).transforms[0].translation.y;
      link_state.request.link_state.pose.position.z = (*it).transforms[0].translation.z;
      link_state.request.link_state.pose.orientation.x = (*it).transforms[0].rotation.x;
      link_state.request.link_state.pose.orientation.y = (*it).transforms[0].rotation.y;
      link_state.request.link_state.pose.orientation.z = (*it).transforms[0].rotation.z;
      link_state.request.link_state.pose.orientation.w = (*it).transforms[0].rotation.w;
      set_link_state_client.call(link_state);
      rate.sleep();
    }

  }

  return true;
}

int main(int argc, char **argv)
{
  ROS_INFO("ok1");


  ros::init(argc, argv, "move_hand");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  move_group_interface = new moveit::planning_interface::MoveGroupInterface("hand");
  ROS_INFO("ok");
  move_group_interface->setPlanningTime(1.0);
  // move_group_interface->setGoalOrientationTolerance(10.0);
  // move_group_interface->setGoalOrientationTolerance(0.1);
  move_group_interface->setMaxVelocityScalingFactor(1.0);
  move_group_interface->setMaxAccelerationScalingFactor(1.0);
  named_targets = move_group_interface->getNamedTargets();

  move_group_interface->clearPathConstraints();
  move_group_interface->clearTrajectoryConstraints();

  set_link_state_client = node_handle.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
  get_link_state_client = node_handle.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  p_scene = new planning_scene::PlanningScene(robot_model);

  // traj_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>(ROBOT_NAME + "_arm_controller/command", 10);
  ros::ServiceServer move_pose_target_service = node_handle.advertiseService("move_hand_pose_target", move_pose_target_server);
  ros::ServiceServer move_named_target_service = node_handle.advertiseService("move_hand_named_target", move_named_target_server);

  ros::Rate loop(50);
  while(ros::ok())
    loop.sleep();

  delete move_group_interface;
  delete p_scene;
  ros::shutdown();
  return 0;
}
