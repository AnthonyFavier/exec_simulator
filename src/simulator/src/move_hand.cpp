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
#include <Eigen/Dense>

std::string ROBOT_NAME = "";

ros::Publisher traj_pub;
moveit::planning_interface::MoveGroupInterface *move_group_interface;
std::vector<std::string> named_targets;
ros::ServiceClient set_link_state_client;
ros::ServiceClient get_link_state_client;

planning_scene::PlanningScene *p_scene;

geometry_msgs::Point init_hand_pose;

geometry_msgs::Quaternion get_rotation_between(Eigen::Vector3d v)
{
  Eigen::Vector3d u;
  // u << 1.0, 0.0, 0.0;
  u << -1.0, 0.0, 0.0;
  float k_cos_theta = u.dot(v);
  float k = sqrt(u.norm()) * v.norm();

  Eigen::Vector4d qe;

  if (k_cos_theta / k == -1)
  {
    // 180 degree rotation around any orthogonal vector
    qe << 0.0, 0.0, 1.0, 0.0;
  }
  else
  {
    Eigen::Vector3d c = u.cross(v);
    qe << k_cos_theta+k, c[0], c[1], c[2];
  }

  qe.normalize();
  geometry_msgs::Quaternion q;
  q.w = qe[0];
  q.x = qe[1];
  q.y = qe[2];
  q.z = qe[3];
  return q;
}

geometry_msgs::Quaternion get_quat(geometry_msgs::Point target)
{
  geometry_msgs::Point a;
  a.x = target.x - init_hand_pose.x;
  a.x = target.y - init_hand_pose.y;
  a.x = target.z - init_hand_pose.z + 3.14159;

  double r = atan2( sqrt(pow(a.y, 2)+pow(a.z,2)), a.x);
  double p = atan2( sqrt(pow(a.z, 2)+pow(a.x,2)), a.y);
  double y = atan2( sqrt(pow(a.x, 2)+pow(a.y,2)), a.z);

  tf2::Quaternion q_tf;
  q_tf.setRPY(r, p, y);

  geometry_msgs::Quaternion q;
  q.x = q_tf.getX();
  q.y = q_tf.getY();
  q.z = q_tf.getZ();
  q.w = q_tf.getW();

  return q;
}

geometry_msgs::Transform interpolate(trajectory_msgs::MultiDOFJointTrajectoryPoint a, trajectory_msgs::MultiDOFJointTrajectoryPoint b, ros::Duration t)
{
  double r = (t - a.time_from_start).toSec()/(b.time_from_start - a.time_from_start).toSec();

  geometry_msgs::Transform atr = a.transforms[0];
  geometry_msgs::Transform btr = b.transforms[0];

  geometry_msgs::Transform dtr;
  dtr.translation.x = btr.translation.x - atr.translation.x;
  dtr.translation.y = btr.translation.y - atr.translation.y;
  dtr.translation.z = btr.translation.z - atr.translation.z;
  dtr.rotation.x = btr.rotation.x - atr.rotation.x;
  dtr.rotation.y = btr.rotation.y - atr.rotation.y;
  dtr.rotation.z = btr.rotation.z - atr.rotation.z;
  dtr.rotation.w = btr.rotation.w - atr.rotation.w;

  geometry_msgs::Transform tr;
  tr.translation.x = atr.translation.x + r * dtr.translation.x;
  tr.translation.y = atr.translation.y + r * dtr.translation.y;
  tr.translation.z = atr.translation.z + r * dtr.translation.z;
  tr.rotation.x = atr.rotation.x + r * dtr.rotation.x;
  tr.rotation.y = atr.rotation.y + r * dtr.rotation.y;
  tr.rotation.z = atr.rotation.z + r * dtr.rotation.z;
  tr.rotation.w = atr.rotation.w + r * dtr.rotation.w;

  return tr;
}

std::string get_transform_str(geometry_msgs::Transform tr)
{
  std::string str;
  

  str += std::to_string(tr.translation.x);
  str += ", ";
  str += std::to_string(tr.translation.y);
  str += ", ";
  str += std::to_string(tr.translation.z);
  // str += ", ";
  // str += std::to_string(tr.rotation.x);
  // str += ", ";
  // str += std::to_string(tr.rotation.y);
  // str += ", ";
  // str += std::to_string(tr.rotation.z);
  // str += ", ";
  // str += std::to_string(tr.rotation.w);
  
  return str;
}

bool move_pose_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
  ROS_INFO("Pose target received.");

  // Get Start Position
  gazebo_msgs::GetLinkState get_link_state;
  get_link_state.request.link_name = "human_hand_link";
  get_link_state_client.call(get_link_state);

  // Set Start State
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
  std::vector<double> joint_values;
  joint_values.push_back(req.pose_target.position.x);
  joint_values.push_back(req.pose_target.position.y);
  joint_values.push_back(req.pose_target.position.z);
  
  if(false)
  {
    geometry_msgs::Point target;
    target.x = req.pose_target.position.x;
    target.y = req.pose_target.position.y;
    target.z = req.pose_target.position.z;
    geometry_msgs::Quaternion q = get_quat(target);
    joint_values.push_back(q.x); // x
    joint_values.push_back(q.y); // y
    joint_values.push_back(q.z); // z
    joint_values.push_back(q.w); // w
  }
  else
  {
    joint_values.push_back(0.0); // x
    joint_values.push_back(0.0); // y
    joint_values.push_back(1.0); // z
    joint_values.push_back(0.0); // w
  }

  
  move_group_interface->setJointValueTarget(joint_values);


  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  res.success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Execute
  if(!res.success)
    ROS_INFO("Failed to plan...");
  else
  {
    // Apply time ratio on traj
    float time_ratio = 0.5; // 1.0=normal speed, 0.5=half speed,  2.0=double speed
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::iterator it;
    for(it = plan.trajectory_.multi_dof_joint_trajectory.points.begin(); it!=plan.trajectory_.multi_dof_joint_trajectory.points.end(); it++)
      (*it).time_from_start = ros::Duration( (*it).time_from_start.toSec() / time_ratio );

    // Execute
    ros::Rate rate(50);
    ros::Time time_start = ros::Time::now();
    gazebo_msgs::SetLinkState link_state;
    link_state.request.link_state.link_name = "human_hand_link";
    // std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::iterator it;
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::iterator it_n;
    it = plan.trajectory_.multi_dof_joint_trajectory.points.begin();
    it_n = it+1;
    // ROS_INFO("a, b =\n%s\n%s", get_transform_str((*it).transforms[0]).c_str(), get_transform_str((*it_n).transforms[0]).c_str());
    while(it_n != plan.trajectory_.multi_dof_joint_trajectory.points.end())
    {
      geometry_msgs::Transform tr = interpolate((*it),(*it_n),ros::Time::now()-time_start);
      // ROS_INFO("\ti = %s", get_transform_str(tr).c_str());
      link_state.request.link_state.pose.position.x = tr.translation.x;
      link_state.request.link_state.pose.position.y = tr.translation.y;
      link_state.request.link_state.pose.position.z = tr.translation.z;
      link_state.request.link_state.pose.orientation.x = tr.rotation.x;
      link_state.request.link_state.pose.orientation.y = tr.rotation.y;
      link_state.request.link_state.pose.orientation.z = tr.rotation.z;
      link_state.request.link_state.pose.orientation.w = tr.rotation.w;
      set_link_state_client.call(link_state);

      if(ros::Time::now()-time_start>(*it_n).time_from_start)
      {
        it = it_n;
        it_n++;
        // if(it_n != plan.trajectory_.multi_dof_joint_trajectory.points.end())
        //   ROS_INFO("a, b =\n%s\n%s", get_transform_str((*it).transforms[0]).c_str(), get_transform_str((*it_n).transforms[0]).c_str());
        // else
        //   ROS_INFO("a, b =\n%s\nEND", get_transform_str((*it).transforms[0]).c_str());
      }
      rate.sleep();
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_hand");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  init_hand_pose.x = 1.38;
  init_hand_pose.y = 0.5;
  init_hand_pose.z = 0.87;

  move_group_interface = new moveit::planning_interface::MoveGroupInterface("hand");
  ROS_INFO("ok");
  move_group_interface->setPlanningTime(1.0);
  named_targets = move_group_interface->getNamedTargets();

  move_group_interface->clearPathConstraints();
  move_group_interface->clearTrajectoryConstraints();

  set_link_state_client = node_handle.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
  get_link_state_client = node_handle.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  p_scene = new planning_scene::PlanningScene(robot_model);

  ros::ServiceServer move_pose_target_service = node_handle.advertiseService("move_hand_pose_target", move_pose_target_server);

  ros::Rate loop(50);
  while(ros::ok())
    loop.sleep();

  delete move_group_interface;
  delete p_scene;
  ros::shutdown();
  return 0;
}
