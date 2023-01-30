/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher p1_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/panda1_arm_controller/command",1);
  ros::Publisher p2_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/panda2_arm_controller/command",1);

  moveit::planning_interface::MoveGroupInterface move_group_interface("panda1_arm");
  move_group_interface.setGoalOrientationTolerance(3.14159265);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(0.9);

  moveit::planning_interface::MoveGroupInterface move_group_interface2("panda2_arm");
  move_group_interface2.setGoalOrientationTolerance(3.14159265);
  move_group_interface2.setMaxVelocityScalingFactor(1.0);
  move_group_interface2.setMaxAccelerationScalingFactor(0.9);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.position.x = 0.622370;
  target_pose1.position.y = 0.304499;
  target_pose1.position.z = 0.409051;
  move_group_interface.setPoseTarget(target_pose1);

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = -1.0;
  target_pose2.position.x = 1.622370;
  target_pose2.position.y = 0.304499;
  target_pose2.position.z = 0.409051;
  move_group_interface2.setPoseTarget(target_pose2);


  // move_group_interface.asyncMove();
  // move_group_interface2.asyncMove();


  moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;
  moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;

  bool rgt_success = (move_group_interface.plan(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  bool lft_success = (move_group_interface2.plan(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  trajectory_msgs::JointTrajectory traj1 = right_arm_plan.trajectory_.joint_trajectory;
  trajectory_msgs::JointTrajectory traj2 = left_arm_plan.trajectory_.joint_trajectory;

  p1_pub.publish(traj1);
  p2_pub.publish(traj2);

  // if(rgt_success && lft_success)
  // {
  //   move_group_interface.asyncExecute(right_arm_plan);
  //   move_group_interface2.asyncExecute(left_arm_plan);
  // }

  ros::shutdown();
  return 0;
}
