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

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface.setGoalOrientationTolerance(3.14159265);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(0.9);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.0;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.position.x = 0.622370;
  target_pose1.position.y = 0.304499;
  target_pose1.position.z = 0.409051;
  // target_pose1.position.x = 0.28;
  // target_pose1.position.y = -0.2;
  // target_pose1.position.z = 0.5;
  move_group_interface.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  // move_group_interface.execute(my_plan);

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute pattern shown above
  // and should be preferred. Note that the pose goal we had set earlier is still active,
  // so the robot will try to move to that goal.

  move_group_interface.move();

// ##################################################################################### //

//   // Adding objects to the environment
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // First let's plan to another simple goal with no objects in the way.
//   move_group_interface.setStartState(*move_group_interface.getCurrentState());
//   geometry_msgs::Pose another_pose;
//   another_pose.orientation.x = 1.0;
//   another_pose.position.x = 0.7;
//   another_pose.position.y = 0.0;
//   another_pose.position.z = 0.59;
//   move_group_interface.setPoseTarget(another_pose);

//   success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("next step");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_clear_path.gif
//   //    :alt: animation showing the arm moving relatively straight toward the goal
//   //
//   // Now let's define a collision object ROS message for the robot to avoid.
//   moveit_msgs::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group_interface.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object.id = "box1";

//   // Define a box to add to the world.
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 0.1;
//   primitive.dimensions[primitive.BOX_Y] = 1.5;
//   primitive.dimensions[primitive.BOX_Z] = 0.5;

//   // Define a pose for the box (specified relative to frame_id)
//   geometry_msgs::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.5;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 0.25;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   ROS_INFO_NAMED("tutorial", "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//   visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//   // Now when we plan a trajectory it will avoid the obstacle
//   success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//   visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//   //    :alt: animation showing the arm moving avoiding the new obstacle
//   //
//   // Attaching objects to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // You can attach objects to the robot, so that it moves with the robot geometry.
//   // This simulates picking up the object for the purpose of manipulating it.
//   // The motion planning should avoid collisions between the two objects as well.
//   moveit_msgs::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper
//   object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
//   geometry_msgs::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.2;

//   // First, we add the object to the world (without using a vector)
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);

//   // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
//   // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
//   ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
//   move_group_interface.attachObject(object_to_attach.id, "panda_hand");

//   visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

//   // Replan, but now with the object in hand.
//   move_group_interface.setStartStateToCurrentState();
//   success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look something like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_attached_object.gif
//   //    :alt: animation showing the arm moving differently once the object is attached
//   //
//   // Detaching and Removing Objects
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Now, let's detach the cylinder from the robot's gripper.
//   ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//   move_group_interface.detachObject(object_to_attach.id);

//   // Show text in RViz of status
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

//   // Now, let's remove the objects from the world.
//   ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   object_ids.push_back(object_to_attach.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
