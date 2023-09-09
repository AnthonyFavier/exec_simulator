/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

/**
 * @file
 *
 * @brief example on how to subscribe to an image topic and how to make the robot look towards a given direction
 *
 * How to test this application:
 *
 * 1) Launch the application:
 *
 *   $ rosrun tiago_tutorials look_to_point
 *
 * 2) Click on image pixels to make TIAGo look towards that direction
 *
 */

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include <std_srvs/Empty.h>

#include "gazebo_msgs/GetModelState.h"
#include <geometry_msgs/Point.h>
#include "sim_msgs/HeadCmd.h"

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string cameraFrame     = "/xtion_rgb_optical_frame";

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

geometry_msgs::Point g_camera_pose;
geometry_msgs::Point g_stack_pose;

ros::ServiceClient g_get_model_state;

bool new_cmd(false);

#define RATE_FOLLOW 2

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Create a ROS action client to move TIAGo's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");

  ROS_INFO("Action client to head controller created.");
}

void look_to_point(geometry_msgs::Point p)
{
  geometry_msgs::PointStamped pointStamped;

  // pointStamped.header.frame_id = cameraFrame;
  pointStamped.header.frame_id = "world";
  pointStamped.point.x = p.x;
  pointStamped.point.y = p.y;
  pointStamped.point.z = p.z;   

  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.75;
  goal.target = pointStamped;

  pointHeadClient->sendGoal(goal);
  ros::Duration(0.5).sleep();
}

void follow_point(geometry_msgs::Point p)
{
  new_cmd = false;

  ros::Rate loop(RATE_FOLLOW);
  while(!new_cmd && ros::ok())
  {
    look_to_point(p);

    ros::spinOnce();
    loop.sleep();
  }
}

void look_at_obj(std::string obj_name)
{
  // get obj_pose
  gazebo_msgs::GetModelState srv;
  srv.request.model_name = obj_name;
  g_get_model_state.call(srv);
  geometry_msgs::Point obj_pose = srv.response.pose.position;

  // call look_to_point
  look_to_point(obj_pose);
}

void follow_obj(std::string obj_name)
{
  new_cmd = false;

  ros::Rate loop(RATE_FOLLOW);
  while(!new_cmd && ros::ok())
  {
    look_at_obj(obj_name);

    ros::spinOnce();
    loop.sleep();
  }
}

void look_at_stack()
{
  look_to_point(g_stack_pose);
}

void follow_stack()
{
  new_cmd = false;

  ros::Rate loop(RATE_FOLLOW);
  while(!new_cmd && ros::ok())
  {
    look_at_stack();

    ros::spinOnce();
    loop.sleep();
  }
}

void look_at_human()
{
  look_to_point(g_camera_pose);
}

void follow_human()
{
  new_cmd = false;

  ros::Rate loop(RATE_FOLLOW);
  while(!new_cmd && ros::ok())
  {
    look_at_human();

    ros::spinOnce();
    loop.sleep();
  }
}

void look_at_h_hand()
{
  look_at_obj("human_hand");
}

void follow_h_hand()
{
  new_cmd = false;

  ros::Rate loop(RATE_FOLLOW);
  while(!new_cmd && ros::ok())
  {
    look_at_h_hand();

    ros::spinOnce();
    loop.sleep();
  }
}

void be_passive()
{
  look_at_human();
  /* OR */
  // reset();
}

void reset()
{
  geometry_msgs::PointStamped pointStamped;

  // Find the right frame near robot head, as to have an axe parallel to x world axe
  pointStamped.header.frame_id = "torso_lift_link";
  pointStamped.point.x = 0.5;
  pointStamped.point.y = 0.0;
  pointStamped.point.z = 0.18;

  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.75;
  goal.target = pointStamped;

  pointHeadClient->sendGoal(goal);
  ros::Duration(0.5).sleep();
}

void cmd_cb(sim_msgs::HeadCmd cmd)
{
  ROS_INFO("Head Cmd received.");
  new_cmd = true;

  switch (cmd.type)
  {
  case sim_msgs::HeadCmd::LOOK_AT_POSE:
    look_to_point(cmd.pose);
    break;

  case sim_msgs::HeadCmd::FOLLOW_POSE:
    follow_point(cmd.pose);
    break;

  case sim_msgs::HeadCmd::LOOK_AT_OBJ:
    look_at_obj(cmd.obj_name);
    break;

  case sim_msgs::HeadCmd::FOLLOW_OBJ:
    follow_obj(cmd.obj_name);
    break;

  case sim_msgs::HeadCmd::LOOK_AT_STACK:
    look_at_stack();
    break;

  case sim_msgs::HeadCmd::FOLLOW_STACK:
    follow_stack();
    break;

  case sim_msgs::HeadCmd::LOOK_AT_HUMAN:
    look_at_human();
    break;
    
  case sim_msgs::HeadCmd::FOLLOW_HUMAN:
    follow_human();
    break;
    
  case sim_msgs::HeadCmd::LOOK_AT_H_HAND:
    look_at_h_hand();
    break;
    
  case sim_msgs::HeadCmd::FOLLOW_H_HAND:
    follow_h_hand();
    break;

  case sim_msgs::HeadCmd::BE_PASSIVE:
    be_passive();
    break;

  case sim_msgs::HeadCmd::RESET:
    reset();
    break;
  
  default:
    break;
  }
}

bool head_ready_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "look_to_point");

  // g_camera_pose
  g_camera_pose.x = 2.237550;
  g_camera_pose.y = 0.0;
  g_camera_pose.z = 2.492110;

  // g_stack_pose
  g_stack_pose.x = 0.86;
  g_stack_pose.y = 0.34;
  g_stack_pose.z = 0.85;

  ROS_INFO("Starting look_to_point application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  ros::Subscriber cmd_sub = nh.subscribe("/test_tiago_head", 1, cmd_cb);
  ros::ServiceServer head_started = nh.advertiseService("/tiago_head_ready", head_ready_srv);

  g_get_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // Create a point head action client to move the TIAGo's head
  createPointHeadClient( pointHeadClient );

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  return EXIT_SUCCESS;
}
