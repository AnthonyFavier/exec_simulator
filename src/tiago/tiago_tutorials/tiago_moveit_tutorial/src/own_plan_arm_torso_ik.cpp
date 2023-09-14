/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <sim_msgs/MoveArm.h>

// Std C++ headers
#include <string>
#include <vector>

moveit::planning_interface::MoveGroupInterface* group_arm_torso;
ros::Publisher start_moving_pub;
std::vector<std::string> named_targets;
std_msgs::Empty e_msg;  

std::string opti_planner_name = "RRTstarkConfigDefault";
double opti_planning_time = 0.6;
std::string fast_planner_name = "SBLkConfigDefault";
double fast_planning_time = 5.0;

void use_opti_planner()
{
  group_arm_torso->setPlannerId(opti_planner_name);
  group_arm_torso->setPlanningTime(opti_planning_time);
}

void use_fast_planner()
{
  group_arm_torso->setPlannerId(fast_planner_name);
  group_arm_torso->setPlanningTime(fast_planning_time);
}

void plan_and_execute()
{
  // Set start state
  group_arm_torso->setStartStateToCurrentState();

  // PLAN //
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // Try quickly with opti planner
  ROS_INFO("Start opti planning...");
  use_opti_planner();
  bool success = bool(group_arm_torso->plan(plan));
  double time_to_plan = plan.planning_time_;
  if ( !success )
  {
    // Try with non opti fast planner
    ROS_WARN("Start fast planning...");
    use_fast_planner();
    moveit::planning_interface::MoveGroupInterface::Plan fast_plan;
    success = bool(group_arm_torso->plan(fast_plan));
    time_to_plan += fast_plan.planning_time_;
    plan = fast_plan;
  }
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << time_to_plan << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();
  start_moving_pub.publish(e_msg);
  if (!bool(group_arm_torso->execute(plan)))
    throw std::runtime_error("Error executing plan");
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}

bool move_named_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
  if (std::find(named_targets.begin(), named_targets.end(), req.named_target) == named_targets.end())
	{
		ROS_ERROR("Named target not existing!");
		res.success = false;
	}
	else
	{
		// Set goal pose
		group_arm_torso->setNamedTarget(req.named_target);

    plan_and_execute();
    res.success = true;
    group_arm_torso->clearPoseTargets();
	}
	return true;
}

bool move_pose_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
	ROS_INFO("Pose target received.");

	// Set goal pose
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
  goal_pose.pose.position = req.pose_target.position;
  goal_pose.pose.orientation.x = 0;
  goal_pose.pose.orientation.y = 0;
  goal_pose.pose.orientation.z = 0;
  goal_pose.pose.orientation.w = 1;
	group_arm_torso->setPoseTarget(req.pose_target);

  plan_and_execute();
  res.success = true;
  group_arm_torso->clearPoseTargets();

	return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_arm_torso_ik");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  group_arm_torso = new moveit::planning_interface::MoveGroupInterface("arm_torso");
  group_arm_torso->setPoseReferenceFrame("world"); 
  group_arm_torso->setMaxVelocityScalingFactor(1.0);
	group_arm_torso->setMaxAccelerationScalingFactor(0.5);
	group_arm_torso->setGoalOrientationTolerance(10.0);
	group_arm_torso->setGoalPositionTolerance(0.01);
  use_opti_planner();

	named_targets = group_arm_torso->getNamedTargets();

  start_moving_pub = nh.advertise<std_msgs::Empty>("/r_start_moving", 1);
	ros::ServiceServer move_pose_target_service = nh.advertiseService("move_pose_target", move_pose_target_server);
	ros::ServiceServer move_named_target_service = nh.advertiseService("move_named_target", move_named_target_server);

  ros::Rate loop(10);
  while(ros::ok())
    loop.sleep();

  delete group_arm_torso;
  spinner.stop();
  return EXIT_SUCCESS;
}
