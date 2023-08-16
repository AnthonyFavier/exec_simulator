#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/String.h>
#include <algorithm>
#include "sim_msgs/MoveArm.h"
#include "moveit_msgs/Constraints.h"
#include <std_msgs/Empty.h>

std::string ROBOT_NAME = "";

ros::Publisher traj_pub;
std_msgs::Empty e_msg;
ros::Publisher start_moving_pub;
moveit::planning_interface::MoveGroupInterface *move_group_interface;
std::vector<std::string> named_targets;

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
		move_group_interface->setNamedTarget(req.named_target);

		// Plan
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		res.success = success;

		// Execute
		if (!success)
			ROS_INFO("Failed to plan...");
		else
		{
			start_moving_pub.publish(e_msg);
			move_group_interface->execute(plan);
		}
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
	bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	res.success = success;

	// Execute
	if (!success)
		ROS_INFO("Failed to plan...");
	else
	{
		start_moving_pub.publish(e_msg);
		move_group_interface->execute(plan);
	}

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
	move_group_interface->setPlanningTime(1.0);
	move_group_interface->setGoalOrientationTolerance(10.0);
	// move_group_interface->setGoalOrientationTolerance(0.1);
	move_group_interface->setMaxVelocityScalingFactor(1.0);
	move_group_interface->setMaxAccelerationScalingFactor(1.0);
	named_targets = move_group_interface->getNamedTargets();

	move_group_interface->clearPathConstraints();
	move_group_interface->clearTrajectoryConstraints();

	traj_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>(ROBOT_NAME + "_arm_controller/command", 10);
	start_moving_pub = node_handle.advertise<std_msgs::Empty>("/r_start_moving", 1);
	ros::ServiceServer move_pose_target_service = node_handle.advertiseService("move_pose_target", move_pose_target_server);
	ros::ServiceServer move_named_target_service = node_handle.advertiseService("move_named_target", move_named_target_server);

	ros::Rate loop(50);
	while (ros::ok())
		loop.sleep();

	delete move_group_interface;
	ros::shutdown();
	return 0;
}
