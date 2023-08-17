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
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

std::string ROBOT_NAME = "";

ros::Publisher traj_pub;
moveit::planning_interface::MoveGroupInterface *move_group_interface;
std::vector<std::string> named_targets;
ros::ServiceClient set_link_state_client;
ros::ServiceClient get_link_state_client;

planning_scene::PlanningScene *p_scene;

geometry_msgs::Point init_hand_pose;
ros::Publisher start_moving_pub;
std_msgs::Empty e_msg;

double goal_tolerance;


bool pass_signal_server(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
	ROS_INFO("Hand Pass Signal");

	// Get Start Position
	gazebo_msgs::GetLinkState get_link_state;
	get_link_state.request.link_name = "human_hand_link";
	get_link_state_client.call(get_link_state);

	gazebo_msgs::SetLinkState link_state;
	link_state.request.link_state.link_name = "human_hand_link";
	link_state.request.link_state.pose = get_link_state.response.link_state.pose;

	double rotation = 0.0;
	tf2::Quaternion q;
	int nb_steps(20);
	ros::Duration dur(0.7);

	// Start moving
	start_moving_pub.publish(e_msg);
	for (int i = 0; i < nb_steps; i++)
	{
		if (i < nb_steps / 2)
			rotation -= 1.570796327 / (nb_steps / 2);
		else
			rotation += 1.570796327 / (nb_steps / 2);

		q.setRPY(0, rotation, -3.14159265356);
		q.normalize();

		link_state.request.link_state.pose.orientation.x = q.getX();
		link_state.request.link_state.pose.orientation.y = q.getY();
		link_state.request.link_state.pose.orientation.z = q.getZ();
		link_state.request.link_state.pose.orientation.w = q.getW();

		set_link_state_client.call(link_state);

		ros::Duration(dur.toSec() / nb_steps).sleep();
	}

	return true;
}

bool move_pose_target_server(sim_msgs::MoveArmRequest &req, sim_msgs::MoveArmResponse &res)
{
	ROS_INFO("Pose target received.");
	res.success = true;

	// Get Start Position
	gazebo_msgs::GetLinkState get_link_state;
	get_link_state.request.link_name = "human_hand_link";
	get_link_state_client.call(get_link_state);
	geometry_msgs::Pose start_pose = get_link_state.response.link_state.pose;
	// std::cout << "start hand pose : " << start_pose.position.x <<","<< start_pose.position.y <<","<< start_pose.position.z << std::endl;

	// Set goal pose
	geometry_msgs::Pose goal_pose;
	goal_pose.position = req.pose_target.position;
	goal_pose.orientation.w = 0.0;
	goal_pose.orientation.x = 0.0;
	goal_pose.orientation.y = 0.0;
	goal_pose.orientation.z = 1.0;
	// std::cout << "goal hand pose : " << goal_pose.position.x <<","<< goal_pose.position.y <<","<< goal_pose.position.z << std::endl;

	// Compute direction vector
	geometry_msgs::Pose dir_vec;
	dir_vec.position.x = goal_pose.position.x - start_pose.position.x;
	dir_vec.position.y = goal_pose.position.y - start_pose.position.y;
	dir_vec.position.z = goal_pose.position.z - start_pose.position.z;
	// std::cout << "dir vec before : " << dir_vec.position.x <<","<< dir_vec.position.y <<","<< dir_vec.position.z << std::endl;

	// Check if already at goal position
	if(abs(dir_vec.position.x) <= goal_tolerance
	&& abs(dir_vec.position.y) <= goal_tolerance
	&& abs(dir_vec.position.z) <= goal_tolerance)
	{
		// std::cout << "Already at goal position" << std::endl;
		return true;
	}

	// Normalize direction vector
	double norm = sqrt(pow(dir_vec.position.x,2) + pow(dir_vec.position.y,2) + pow(dir_vec.position.z,2));
	// std::cout << "norm : " << norm << std::endl;
	dir_vec.position.x /= norm;
	dir_vec.position.y /= norm;
	dir_vec.position.z /= norm;
	// std::cout << "dir vec normalized : " << dir_vec.position.x <<","<< dir_vec.position.y <<","<< dir_vec.position.z << std::endl;

	// Define speed
	double desired_speed = 0.3; // m
	ros::Rate loop(50); // Hz
	double step = desired_speed * loop.expectedCycleTime().toSec();
	// std::cout << "step : " << step << std::endl;

	// Move
	gazebo_msgs::SetLinkState link_state;
	link_state.request.link_state.link_name = "human_hand_link";
	link_state.request.link_state.pose.orientation.w = goal_pose.orientation.w;
	link_state.request.link_state.pose.orientation.x = goal_pose.orientation.x;
	link_state.request.link_state.pose.orientation.y = goal_pose.orientation.y;
	link_state.request.link_state.pose.orientation.z = goal_pose.orientation.z;
	geometry_msgs::Pose curr_pose = start_pose;
	start_moving_pub.publish(e_msg);
	bool goal_reached = false;
	while(ros::ok() && !goal_reached)
	{
		// compute next pose
		curr_pose.position.x += dir_vec.position.x * step;
		curr_pose.position.y += dir_vec.position.y * step;
		curr_pose.position.z += dir_vec.position.z * step;

		// check if the goal pose is reached or passed
		goal_reached = dir_vec.position.x>=0 && curr_pose.position.x>=goal_pose.position.x
					|| dir_vec.position.y>=0 && curr_pose.position.y>=goal_pose.position.y
					|| dir_vec.position.z>=0 && curr_pose.position.z>=goal_pose.position.z
					|| dir_vec.position.x<=0 && curr_pose.position.x<=goal_pose.position.x
					|| dir_vec.position.y<=0 && curr_pose.position.y<=goal_pose.position.y
					|| dir_vec.position.z<=0 && curr_pose.position.z<=goal_pose.position.z;

		// backtrack if passed the goal
		if(goal_reached)
			curr_pose = goal_pose;

		// move hand link
		link_state.request.link_state.pose = curr_pose;
		set_link_state_client.call(link_state);

		loop.sleep();
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

	goal_tolerance = 0.01;

	start_moving_pub = node_handle.advertise<std_msgs::Empty>("/h_start_moving", 1);
	set_link_state_client = node_handle.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
	get_link_state_client = node_handle.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

	ros::ServiceServer move_pose_target_service = node_handle.advertiseService("move_hand_pose_target", move_pose_target_server);
	ros::ServiceServer pass_signal_service = node_handle.advertiseService("move_hand_pass_signal", pass_signal_server);

	ros::Rate loop(50);
	while (ros::ok())
		loop.sleep();

	ros::shutdown();
	return 0;
}
