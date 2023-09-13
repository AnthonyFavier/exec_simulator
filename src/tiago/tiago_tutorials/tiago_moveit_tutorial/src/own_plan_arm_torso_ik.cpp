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

moveit::planning_interface::MoveGroupInterface* group_arm_torso_opti;
moveit::planning_interface::MoveGroupInterface* group_arm_torso_fast;
ros::Publisher start_moving_pub;
std::vector<std::string> named_targets;
std_msgs::Empty e_msg;

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
		group_arm_torso_opti->setNamedTarget(req.named_target);
		group_arm_torso_fast->setNamedTarget(req.named_target);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = bool(group_arm_torso_opti->plan(my_plan));
    bool fast = false;
    if ( !success )
    {
      fast = true;
      ROS_WARN("Opti plan not found, planning with fast solver.");
      success = bool(group_arm_torso_fast->plan(my_plan));
    }
    if ( !success )
      throw std::runtime_error("No plan found");
    double time = my_plan.planning_time_;
    if(fast)
      time += group_arm_torso_opti->getPlanningTime();
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();
    start_moving_pub.publish(e_msg);
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso_opti->execute(my_plan);
    if (!bool(e))
      throw std::runtime_error("Error executing plan");
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    res.success = success;
	}
	return true;
}

bool move_pose_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
	ROS_INFO("Pose target received.");

  // Set start state
  group_arm_torso_opti->setStartStateToCurrentState();
  group_arm_torso_fast->setStartStateToCurrentState();

	// Set goal pose
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
  goal_pose.pose.position = req.pose_target.position;
  goal_pose.pose.orientation.x = 0;
  goal_pose.pose.orientation.y = 0;
  goal_pose.pose.orientation.z = 0;
  goal_pose.pose.orientation.w = 1;
	group_arm_torso_opti->setPoseTarget(req.pose_target);
	group_arm_torso_fast->setPoseTarget(req.pose_target);

	// Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = bool(group_arm_torso_opti->plan(my_plan));
    bool fast = false;
  if ( !success )
  {
      fast = true;
      ROS_WARN("Opti plan not found, planning with fast solver.");
      success = bool(group_arm_torso_fast->plan(my_plan));
  }
  if ( !success )
    throw std::runtime_error("No plan found");
  double time = my_plan.planning_time_;
  if(fast)
    time += group_arm_torso_opti->getPlanningTime();
  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

	// Execute the plan
  ros::Time start = ros::Time::now();
  start_moving_pub.publish(e_msg);
  moveit::planning_interface::MoveItErrorCode e = group_arm_torso_opti->execute(my_plan);
  if (!bool(e))
    throw std::runtime_error("Error executing plan");
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  res.success = true;

	return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_arm_torso_ik");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //select group of joints
  group_arm_torso_opti = new moveit::planning_interface::MoveGroupInterface("arm_torso");
  group_arm_torso_fast = new moveit::planning_interface::MoveGroupInterface("arm_torso");
  //choose your preferred planner
  group_arm_torso_opti->setPlannerId("RRTstarkConfigDefault");
  group_arm_torso_opti->setPoseReferenceFrame("world"); 
  group_arm_torso_opti->setMaxVelocityScalingFactor(1.0);
	group_arm_torso_opti->setMaxAccelerationScalingFactor(0.5);
  group_arm_torso_opti->setPlanningTime(0.5);
	group_arm_torso_opti->setGoalOrientationTolerance(10.0);
	group_arm_torso_opti->setGoalPositionTolerance(0.01);

  group_arm_torso_fast->setPlannerId("SBLkConfigDefault");
  group_arm_torso_fast->setPoseReferenceFrame("world"); 
	group_arm_torso_fast->setGoalOrientationTolerance(10.0);
	group_arm_torso_fast->setGoalPositionTolerance(0.01);

	named_targets = group_arm_torso_opti->getNamedTargets();

  start_moving_pub = nh.advertise<std_msgs::Empty>("/r_start_moving", 1);
	ros::ServiceServer move_pose_target_service = nh.advertiseService("move_pose_target", move_pose_target_server);
	ros::ServiceServer move_named_target_service = nh.advertiseService("move_named_target", move_named_target_server);

  ros::Rate loop(10);
  while(ros::ok())
    loop.sleep();

  delete group_arm_torso_opti;
  spinner.stop();
  return EXIT_SUCCESS;
}
